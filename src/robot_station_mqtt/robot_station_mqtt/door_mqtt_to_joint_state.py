#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
import paho.mqtt.client as mqtt


class DoorMqttToJointStateNode(Node):
    """
    订阅 MQTT 的窗户开合百分比,映射为 prismatic 关节位置并发布到 /joint_states。
    可配置参数:
      - mqtt_host: MQTT Broker 地址(test: 172.17.239.190)
      - mqtt_port: MQTT Broker 端口(default:1883)
      - mqtt_topic: 订阅主题(test:test/yimin)
      - joint_name: 关节名称(test:${prefix}ct_left_door_joint,与 URDF 保持一致)
      - max_travel: 最大滑动距离(默认:0.5,对应 URDF 上限)
    支持 payload 为纯数字字符串(如 "25"、"75.0"),或 JSON(如 {"percent": 25})。
    """

    def __init__(self):
        super().__init__('door_mqtt_to_joint_state')

        # 声明参数
        self.declare_parameter(
            'prefix', '', 
            ParameterDescriptor(description='Prefix for joint names')) 
        self.declare_parameter(
            'mqtt_host', '172.17.239.190',
            ParameterDescriptor(description='MQTT broker host'))
        self.declare_parameter(
            'mqtt_port', 1883,
            ParameterDescriptor(description='MQTT broker port'))
        self.declare_parameter(
            'mqtt_topic', 'test/door/distance',
            ParameterDescriptor(description='MQTT topic for door distance'))
        self.declare_parameter(
            'left_joint_name', 'ct_left_door_joint',
            ParameterDescriptor(description='Left joint name to publish'))
        self.declare_parameter(
            'right_joint_name', 'ct_right_door_joint',
            ParameterDescriptor(description='Right joint name to publish'))
        self.declare_parameter(
            'max_travel', 0.5,
            ParameterDescriptor(description='Max prismatic travel in meters'))
        self.declare_parameter(
            'mimic_multiplier', 1.0,
            ParameterDescriptor(description='Mimic joint multiplier'))
        self.declare_parameter(
            'mimic_offset', 0.0,
            ParameterDescriptor(description='Mimic joint offset'))
        self.declare_parameter(
            'mqtt_username', 'user1',
            ParameterDescriptor(description='MQTT username (empty for no auth)'))
        self.declare_parameter(
            'mqtt_password', '12345',
            ParameterDescriptor(description='MQTT password'))

        # 读取参数
        self.mqtt_host = self.get_parameter('mqtt_host').get_parameter_value().string_value
        self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
        self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value

        prefix = self.get_parameter('prefix').get_parameter_value().string_value
        left_joint_name = self.get_parameter('left_joint_name').get_parameter_value().string_value
        right_joint_name = self.get_parameter('right_joint_name').get_parameter_value().string_value
        self.left_joint_name = f'{prefix}{left_joint_name}'
        self.right_joint_name = f'{prefix}{right_joint_name}'

        self.max_travel = self.get_parameter('max_travel').get_parameter_value().double_value
        self.mimic_multiplier = self.get_parameter('mimic_multiplier').get_parameter_value().double_value
        self.mimic_offset = self.get_parameter('mimic_offset').get_parameter_value().double_value
        self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
        self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value

        # 发布 joint_states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # MQTT 客户端与线程
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        self._mqtt_thread: Optional[threading.Thread] = None

        if self.mqtt_username:
            try:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
                self.get_logger().info(f'MQTT authentication enabled for user: {self.mqtt_username}')
            except Exception as e:
                self.get_logger().error(f'Failed to set MQTT authentication: {e}')

        # 连接 MQTT
        try:
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        except Exception as e:
            self.get_logger().error(f'Failed to connect MQTT broker {self.mqtt_host}:{self.mqtt_port} - {e}')

        # 启动后台线程处理 MQTT loop
        self._mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever, daemon=True)
        self._mqtt_thread.start()
        self.get_logger().info(
            f'MQTT client started. Subscribing to topic "{self.mqtt_topic}" on {self.mqtt_host}:{self.mqtt_port}')

    # MQTT 连接回调
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected successfully')
            try:
                client.subscribe(self.mqtt_topic, qos=0)
            except Exception as e:
                self.get_logger().error(f'Failed to subscribe topic {self.mqtt_topic}: {e}')
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')

    # MQTT 消息回调
    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8').strip()
        percent = self._parse_percent(payload)
        if percent is None:
            self.get_logger().warn(f'Invalid payload "{payload}", expected number or JSON with "percent"')
            return

        # 边界夹紧到 [0, 100]
        percent = max(0.0, min(100.0, percent))
        left_position = (percent / 100.0) * self.max_travel  # 映射到 [0, max_travel]
        right_position = left_position * self.mimic_multiplier + self.mimic_offset

        # 发布 JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint_name, self.right_joint_name]
        js.position = [left_position, right_position]
        # 可选:速度/努力暂不使用
        self.pub.publish(js)

        self.get_logger().info(
            f'Received percent={percent:.2f}% -> '
            f'left_position={left_position:.3f} m, '
            f'right_position={right_position:.3f} m')

    @staticmethod
    def _parse_percent(text: str) -> Optional[float]:
        # 支持纯数字与 JSON 格式
        try:
            # 直接解析为浮点
            return float(text)
        except ValueError:
            pass

        try:
            data = json.loads(text)
            if isinstance(data, dict) and 'percent' in data:
                return float(data['percent'])
        except Exception:
            pass

        return None


def main(args=None):
    rclpy.init(args=args)
    node = DoorMqttToJointStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down mqtt_to_joint_state node')
    finally:
        # 清理资源
        try:
            node.mqtt_client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
