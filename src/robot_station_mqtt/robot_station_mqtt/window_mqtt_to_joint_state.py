#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
window_mqtt_to_joint_state.py
Subscribe to MQTT state messages (Open/Closed/Uninitialized), map them to joint positions and publish to /joint_states.
Suitable for sliding windows that only have discrete states.
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
    Subscribe to MQTT window state messages, map them to prismatic joint positions and publish to /joint_states.
    
    Supported states:
      - "Open" → 100% (fully Open, position = max_travel)
      - "Closed" → 0% (Closed, position = 0.0)
      - "Uninitialized" → 0% (Uninitialized, position = 0.0)
    
    Configurable parameters:
      - mqtt_host: MQTT Broker address (default: localhost)
      - mqtt_port: MQTT Broker port (default: 1883)
      - mqtt_username: MQTT username (default: empty string, means no auth)
      - mqtt_password: MQTT password (default: empty string)
      - mqtt_topic: subscription topic (default: test/window/state)
      - prefix: joint name prefix (default: empty string, for multi-robot scenarios)
      - joint_name: base joint name (default: window_slide_joint, actual name is prefix + joint_name)
      - max_travel: maximum sliding distance (default: 0.5, corresponds to URDF limit)
    
    Supported payload formats:
      - Plain string: "Open", "Closed", "Uninitialized"
      - JSON: {"state": "Open"} or {"status": "Closed"}
    """

    # Mapping from state to percentage
    STATE_MAP = {
        'Open': 100.0,
        'Closed': 0.0,
        'Uninitialized': 0.0
    }

    def __init__(self):
        super().__init__('window_mqtt_to_joint_state')

        try:
            # Declare parameters
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
                'mqtt_username', 'user1',
                ParameterDescriptor(description='MQTT username (empty for no auth)'))
            self.declare_parameter(
                'mqtt_password', 
                # '12345', # test
                'crc1574', # nuc at wbk
                ParameterDescriptor(description='MQTT password'))
            self.declare_parameter(
                'mqtt_topic', 
                # 'test/window/state', # test
                'esp32-window-ct/select/status/state', # wbk
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

            # Read parameters
            self.mqtt_host = self.get_parameter('mqtt_host').get_parameter_value().string_value
            self.mqtt_port = self.get_parameter('mqtt_port').get_parameter_value().integer_value
            self.mqtt_username = self.get_parameter('mqtt_username').get_parameter_value().string_value
            self.mqtt_password = self.get_parameter('mqtt_password').get_parameter_value().string_value
            self.mqtt_topic = self.get_parameter('mqtt_topic').get_parameter_value().string_value
            self.max_travel = self.get_parameter('max_travel').get_parameter_value().double_value

            prefix = self.get_parameter('prefix').get_parameter_value().string_value
            base_joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
            self.joint_name = f'{prefix}{base_joint_name}'
            
            
            # Log joint name information
            if prefix:
                self.get_logger().info(
                    f'Using prefixed joint name: "{self.joint_name}" '
                    f'(prefix: "{prefix}", base: "{base_joint_name}")')
            else:
                self.get_logger().info(f'Using joint name: "{self.joint_name}" (no prefix)')

            # Log supported states
            self.get_logger().info(f'Supported states: {list(self.STATE_MAP.keys())}')
            self.get_logger().info(f'State mapping: Open=100%, Closed=0%, Uninitialized=0%')

            # Publish joint_states
            self.pub = self.create_publisher(JointState, '/joint_states', 10)

            # MQTT client and thread
            self.mqtt_client = mqtt.Client()
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_message = self._on_message
            self._mqtt_thread: Optional[threading.Thread] = None

            # Set username and password (if provided)
            if self.mqtt_username:
                self.mqtt_client.username_pw_set(self.mqtt_username, self.mqtt_password)
                self.get_logger().info(f'MQTT authentication enabled for user: {self.mqtt_username}')

            # Connect to MQTT
            self.mqtt_client.connect(self.mqtt_host, self.mqtt_port, keepalive=60)
            
            # Start background thread to handle MQTT loop
            self._mqtt_thread = threading.Thread(target=self.mqtt_client.loop_forever, daemon=True)
            self._mqtt_thread.start()
            self.get_logger().info(
                f'MQTT client started. Subscribing to topic "{self.mqtt_topic}" '
                f'on {self.mqtt_host}:{self.mqtt_port}')

        except Exception as e:
            self.get_logger().error(f'Failed to initialize WindowMqttToJointStateNode: {e}')
            raise

    def _on_connect(self, client, userdata, flags, rc):
        """MQTT connection callback"""
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
        """MQTT message callback"""
        try:
            payload = msg.payload.decode('utf-8').strip()
            state = self._parse_state(payload)
            
            if state is None:
                self.get_logger().warn(
                    f'Invalid payload "{payload}", expected one of: {list(self.STATE_MAP.keys())}')
                return

            # Get the percentage corresponding to the state (case-insensitive match)
            percent = None
            for key, val in self.STATE_MAP.items():
                if key == state or key.lower() == state.lower():
                    percent = val
                    break
            if percent is None:
                self.get_logger().warn(
                    f'Unknown state "{state}", expected one of: {list(self.STATE_MAP.keys())}')
                return

            # Map to joint position
            position = (percent / 100.0) * self.max_travel

            # Publish JointState
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
        Parse state string
        Supported formats:
          - Plain string: "Open", "Closed", "Uninitialized"
          - JSON: {"state": "Open"} or {"status": "Closed"}
        """
        # Preserve and return the exact string as published when it matches a known state.
        raw = text.strip()
        if not raw:
            return None

        # Direct exact match against known states
        for key in WindowMqttToJointStateNode.STATE_MAP.keys():
            if raw == key:
                return raw

        # Case-insensitive direct match: return the original casing from the payload
        for key in WindowMqttToJointStateNode.STATE_MAP.keys():
            if raw.lower() == key.lower():
                return raw

        # Try parsing JSON and extract 'state' or 'status' without forcing lower-case
        try:
            data = json.loads(text)
            if isinstance(data, dict):
                for field in ('state', 'status'):
                    if field in data:
                        state_value = str(data[field]).strip()
                        # Exact match
                        for key in WindowMqttToJointStateNode.STATE_MAP.keys():
                            if state_value == key:
                                return state_value
                        # Case-insensitive match -> still return original casing
                        for key in WindowMqttToJointStateNode.STATE_MAP.keys():
                            if state_value.lower() == key.lower():
                                return state_value
        except Exception:
            pass

        return None


def main(args=None):
    """Main function"""
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
        # Clean up resources
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