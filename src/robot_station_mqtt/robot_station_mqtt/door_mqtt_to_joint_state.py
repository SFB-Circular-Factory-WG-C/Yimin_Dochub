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
    Subscribe to MQTT door open percentage, map it to prismatic joint positions and publish to /joint_states.
    Configurable parameters:
      - mqtt_host: MQTT broker host (example: 192.168.2.104)
      - mqtt_port: MQTT broker port (default:1883)
      - mqtt_topic: MQTT topic to subscribe to (example: test/door/distance)
      - left_joint_name/right_joint_name: joint names (example: ${prefix}ct_left_door_joint, must match URDF)
      - max_travel: maximum prismatic travel (default:0.5, corresponds to URDF limit)
      - mimic_multiplier/mimic_offset: for right joint mimic behavior
        - mqtt_username/mqtt_password: for MQTT authentication (example: user1/12345)
    Supports payload as plain numeric string (e.g. "25", "75.0"), or JSON (e.g. {"percent": 25}).
    Maps [0%,100%] to [0,max_travel] (m) for left joint, and applies mimic for right joint.
    * Problem is what we get from MQTT is actually the distance from the sensor to the door edge,
    * so 0% means fully closed (about 0.33m), and 100% means fully open (about 0.63m).
    * My idea is to have the sensor/MQTT publisher side convert the distance to door open distance,
    * This conversion should be handled by the sensor/MQTT publisher side later.
    """

    def __init__(self):
        super().__init__('door_mqtt_to_joint_state')

        # Declare parameters
        self.declare_parameter(
            'prefix', '', 
            ParameterDescriptor(description='Prefix for joint names')) 
        self.declare_parameter(
            'mqtt_host', 
            # '192.168.2.104', # test at home
            # '172.17.56.139', # test VM at wbk
            '172.23.253.37', # nuc at wbk
            ParameterDescriptor(description='MQTT broker host'))
        self.declare_parameter(
            'mqtt_port', 
            # 1883, # test
            1884, # nuc at wbk
            ParameterDescriptor(description='MQTT broker port'))
        self.declare_parameter(
            'mqtt_topic', 
            # 'test/door/distance', # test
            'esp32-door-distance-ct-cell/sensor/vl53l0x_distance/state', # wbk
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
            'mqtt_password', 
            # '12345', # test
            'crc1574', # nuc at wbk
            ParameterDescriptor(description='MQTT password'))

        # Read parameters
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

        # Publisher for joint_states
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # MQTT client and thread
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

        # Connect to MQTT
        try:
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
        except Exception as e:
            self.get_logger().error(f'Failed to connect MQTT broker {self.mqtt_host}:{self.mqtt_port} - {e}')

        # Start background thread to handle MQTT loop
        self._mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever, daemon=True)
        self._mqtt_thread.start()
        self.get_logger().info(
            f'MQTT client started. Subscribing to topic "{self.mqtt_topic}" on {self.mqtt_host}:{self.mqtt_port}')

    # MQTT connection callback
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('MQTT connected successfully')
            try:
                client.subscribe(self.mqtt_topic, qos=0)
            except Exception as e:
                self.get_logger().error(f'Failed to subscribe topic {self.mqtt_topic}: {e}')
        else:
            self.get_logger().error(f'MQTT connection failed with code {rc}')

    # MQTT message callback
    def _on_message(self, client, userdata, msg):
        payload = msg.payload.decode('utf-8').strip()
        value = self._parse_percent(payload)
        if value is None:
            self.get_logger().warn(f'Invalid payload "{payload}", expected number or JSON with "percent" or distance')
            return

        # If the publisher sends a distance in meters (e.g. 0.34..0.64),
        # map that to percent where 0.34 -> 100% and 0.64 -> 0%.
        try:
                sensor_min = 0.34
                sensor_max = 0.64
                # Clamp sensor value into expected sensor range
                sensor_val = max(sensor_min, min(sensor_max, float(value)))
                percent = (sensor_max - sensor_val) / (sensor_max - sensor_min) * 100.0
        except Exception:
            self.get_logger().warn(f'Failed to interpret payload value "{value}"')
            return

        # Clamp to [0, 100]
        percent = max(0.0, min(100.0, percent))
        left_position = (percent / 100.0) * self.max_travel  # Map to [0, max_travel]
        right_position = left_position * self.mimic_multiplier + self.mimic_offset

        # Publish JointState
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [self.left_joint_name, self.right_joint_name]
        js.position = [left_position, right_position]
        # Optional: velocity/effort not used for now
        self.pub.publish(js)

        self.get_logger().info(
            f'Received percent={percent:.2f}% -> '
            f'left_position={left_position:.3f} m, '
            f'right_position={right_position:.3f} m')

    @staticmethod
    def _parse_percent(text: str) -> Optional[float]:
        # Supports plain numbers and JSON format
        try:
            # Try parsing as float directly
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
        # Clean up resources
        try:
            node.mqtt_client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()