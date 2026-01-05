#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
window_mqtt_to_joint_state.py
订阅 MQTT 状态消息（open/closed/uninitialized），映射为关节位置并发布到 /joint_states。
适用于只有离散状态的滑动窗户。
"""

import json
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import JointState
import paho.mqtt.client as mqtt


class WindowMqttToJointStateNode(Node):
    """
    订阅 MQTT 窗户状态消息，映射为 prismatic 关节位置并发布到 /joint_states。
    
    支持的状态：
      - "open" → 100% (完全打开，position = max_travel)
      - "closed" → 0% (关闭，position = 0.0)
      - "uninitialized" → 0% (未初始化，position = 0.0)
    
    可配置参数：
      - mqtt_host: MQTT Broker 地址（默认：localhost）
      - mqtt_port: MQTT Broker 端口（默认：1883）
      - mqtt_username: MQTT 用户名（默认：空字符串，表示无需认证）
      - mqtt_password: MQTT 密码（默认：空字符串）
      - mqtt_topic: 订阅主题（默认：home/window/state）
      - prefix: 关节名称前缀（默认：空字符串，用于多机器人场景）
      - joint_name: 关节基础名称（默认：window_slide_joint，实际名称为 prefix + joint_name）
      - max_travel: 最大滑动距离（默认：0.5，对应 URDF 上限）
    
    支持 payload 格式：
      - 纯字符串："open", "closed", "uninitialized"
      - JSON：{"state": "open"} 或 {"status": "closed"}
    """

    # 状态到百分比的映射
    STATE_MAP = {
        'open': 100.0,
        'closed': 0.0,
        'uninitialized': 0.0
    }

    def __init__(self):
        super().__init__('window_mqtt_to_joint_state')

        try:
            # 声明参数
            self.declare_parameter(
                'mqtt_host', '172.17.239.190',
                ParameterDescriptor(description='MQTT broker host'))
            self.declare_parameter(
                'mqtt_port', 1883,
                ParameterDescriptor(description='MQTT broker port'))
            self.declare_parameter(
                'mqtt_username', 'user1',
                ParameterDescriptor(description='MQTT username (empty for no auth)'))
            self.declare_parameter(
                'mqtt_password', '12345',
                ParameterDescriptor(description='MQTT password'))
            self.declare_parameter(
                'mqtt_topic', 'test/window/state',
                ParameterDescriptor(description='MQTT topic for window state'))
            self.declare_parameter(
                'prefix', '',
                ParameterDescriptor(description='Joint name prefix for multi-robot scenarios'))
            self.declare_parameter(
                'joint_name', 'slide_window_joint',
                ParameterDescriptor(description='Base joint name (actual name will be prefix + joint_name)'))
            self.declare_parameter(
                'max_travel', 1.0,
                ParameterDescriptor(description='Max prismatic travel in meters'))

            # 读取参数
            self.mqtt_host = self.get_parameter('mqtt_host').get_parameter_value().string_value
            self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
            self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
            self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
            self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
            self.max_travel = self.get_parameter('max_travel').get_parameter_value().double_value

            prefix = self.get_parameter('prefix').get_parameter_value().string_value
            base_joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
            self.joint_name = f'{prefix}{base_joint_name}'
            
            
            # 记录关节名称信息
            if prefix:
                self.get_logger().info(
                    f'Using prefixed joint name: "{self.joint_name}" '
                    f'(prefix: "{prefix}", base: "{base_joint_name}")')
            else:
                self.get_logger().info(f'Using joint name: "{self.joint_name}" (no prefix)')

            # 记录支持的状态
            self.get_logger().info(f'Supported states: {list(self.STATE_MAP.keys())}')
            self.get_logger().info(f'State mapping: open=100%, closed=0%, uninitialized=0%')

            # 发布 joint_states
            self.pub = self.create_publisher(JointState, '/joint_states', 10)

            # MQTT 客户端与线程
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_message = self._on_message
            self._mqtt_thread: Optional[threading.Thread] = None

            # 设置用户名和密码（如果提供）
            if self.mqtt_username:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
                self.get_logger().info(f'MQTT authentication enabled for user: {self.mqtt_username}')

            # 连接 MQTT
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            
            # 启动后台线程处理 MQTT loop
            self._mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever, daemon=True)
            self._mqtt_thread.start()
            self.get_logger().info(
                f'MQTT client started. Subscribing to topic "{self.mqtt_topic}" '
                f'on {self.mqtt_host}:{self.mqtt_port}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize WindowMqttToJointStateNode: {e}')
            raise

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT 连接回调"""
        try:
            if rc == 0:
                self.get_logger().info('MQTT connected successfully')
                client.subscribe(self.mqtt_topic, qos=0)
                self.get_logger().info(f'Subscribed to topic: {self.mqtt_topic}')
            else:
                self.get_logger().error(f'MQTT connection failed with code {rc}')
        except Exception as e:
            self.get_logger().error(f'Error in _on_connect: {e}')

    def _on_message(self, client, userdata, msg):
        """MQTT 消息回调"""
        try:
            payload = msg.payload.decode('utf-8').strip()
            state = self._parse_state(payload)
            
            if state is None:
                self.get_logger().warn(
                    f'Invalid payload "{payload}", expected one of: {list(self.STATE_MAP.keys())}')
                return

            # 获取状态对应的百分比
            percent = self.STATE_MAP.get(state.lower())
            if percent is None:
                self.get_logger().warn(
                    f'Unknown state "{state}", expected one of: {list(self.STATE_MAP.keys())}')
                return

            # 映射到关节位置
            position = (percent / 100.0) * self.max_travel

            # 发布 JointState
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = [self.joint_name]
            js.position = [position]
            self.pub.publish(js)

            self.get_logger().info(
                f'Received state="{state}" -> {percent:.0f}% -> position={position:.3f} m')

        except Exception as e:
            self.get_logger().error(f'Error in _on_message: {e}')

    @staticmethod
    def _parse_state(text: str) -> Optional[str]:
        """
        解析状态字符串
        支持格式：
          - 纯字符串："open", "closed", "uninitialized"
          - JSON：{"state": "open"} 或 {"status": "closed"}
        """
        # 尝试直接作为状态字符串
        text_lower = text.lower().strip()
        if text_lower in WindowMqttToJointStateNode.STATE_MAP:
            return text_lower

        # 尝试解析 JSON
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                # 支持 "state" 或 "status" 字段
                for key in ['state', 'status']:
                    if key in data:
                        state_value = str(data[key]).lower().strip()
                        if state_value in WindowMqttToJointStateNode.STATE_MAP:
                            return state_value
        except Exception:
            pass

        return None


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    node = None

    try:
        node = WindowMqttToJointStateNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down window_mqtt_to_joint_state node (Ctrl+C pressed)')

    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error in window_mqtt_to_joint_state: {e}')
        else:
            print(f'Failed to create WindowMqttToJointStateNode: {e}')

    finally:
        # 清理资源
        if node:
            try:
                node.mqtt_client.disconnect()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()