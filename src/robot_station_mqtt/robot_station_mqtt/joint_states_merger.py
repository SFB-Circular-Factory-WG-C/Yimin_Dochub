import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMerger(Node):
    """
    Node that merges multiple joint_states sources.
    Subscribes to joint state messages from different sources and publishes a merged
    message to a unified /joint_states topic.
    Supports multi-source control and avoids publisher conflicts.
    """
    
    def __init__(self):
        super().__init__('joint_state_merger')
        
        try:
            # Subscribe to two independent joint_states sources
            self.sub_jsp = self.create_subscription(
                JointState, '/joint_states_manual', self.jsp_callback, 10)
            self.sub_mqtt_door = self.create_subscription(
                JointState, '/joint_states_door', self.mqtt_door_callback, 10)
            self.sub_mqtt_window = self.create_subscription(
                JointState, '/joint_states_window', self.mqtt_window_callback, 10)
            
            # Publish merged joint_states
            self.pub = self.create_publisher(JointState, '/joint_states', 10)
            
            # Cache the latest joint states
            self.manual_joints = {}  # {joint_name: position}
            self.mqtt_door_joints = {}
            self.mqtt_window_joints = {}
            
            self.get_logger().info('Joint State Merger initialized successfully')
            self.get_logger().info('Subscribing to: /joint_states_manual, /joint_states_door, /joint_states_window')
            self.get_logger().info('Publishing merged states to: /joint_states')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Joint State Merger: {e}')
            raise
    
    def jsp_callback(self, msg):
        """Handle messages from joint_state_publisher"""
        try:
            if not msg.name or len(msg.name) != len(msg.position):
                self.get_logger().warn('Invalid manual joint_states message: name/position mismatch')
                return
            
            for i, name in enumerate(msg.name):
                self.manual_joints[name] = msg.position[i]
            
            self.get_logger().debug(f'Received manual joints: {msg.name}')
            self.publish_merged()
            
        except Exception as e:
            self.get_logger().error(f'Error in jsp_callback: {e}')
    
    def mqtt_door_callback(self, msg):
        """Handle messages from the MQTT door node"""
        try:
            if not msg.name or len(msg.name) != len(msg.position):
                self.get_logger().warn('Invalid MQTT joint_states message: name/position mismatch')
                return
            
            for i, name in enumerate(msg.name):
                self.mqtt_door_joints[name] = msg.position[i]
            
            self.get_logger().debug(f'Received MQTT joints: {msg.name}')
            self.publish_merged()
            
        except Exception as e:
            self.get_logger().error(f'Error in mqtt_door_callback: {e}')
    
    def mqtt_window_callback(self, msg):
        """Handle messages from the MQTT window node"""
        try:
            if not msg.name or len(msg.name) != len(msg.position):
                self.get_logger().warn('Invalid MQTT joint_states message: name/position mismatch')
                return
            
            for i, name in enumerate(msg.name):
                self.mqtt_window_joints[name] = msg.position[i]
            
            self.get_logger().debug(f'Received MQTT joints: {msg.name}')
            self.publish_merged()
            
        except Exception as e:
            self.get_logger().error(f'Error in mqtt_window_callback: {e}')
    
    def publish_merged(self):
        """Merge and publish joint states"""
        try:
            # Merge the dictionaries, MQTT has priority (overwrites manual control joints with the same name)
            merged = {**self.manual_joints, **self.mqtt_door_joints, **self.mqtt_window_joints}

            if not merged:
                self.get_logger().debug('No joint states to publish yet')
                return
            
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = list(merged.keys())
            js.position = list(merged.values())
            
            self.pub.publish(js)
            self.get_logger().debug(f'Published merged states for {len(merged)} joints')
            
        except Exception as e:
            self.get_logger().error(f'Error in publish_merged: {e}')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = JointStateMerger()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down Joint State Merger (Ctrl+C pressed)')
            
    except Exception as e:
        if node:
            node.get_logger().error(f'Unexpected error in Joint State Merger: {e}')
        else:
            print(f'Failed to create Joint State Merger node: {e}')
            
    finally:
        # Clean up resources
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()