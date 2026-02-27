#!/usr/bin/env python3
"""
Map Publisher Node
==================
Publishes a pre-generated occupancy grid map from a PGM/YAML file to /map topic.
This allows RViz to display static maps without running SLAM or map_server.

Usage
-----
  ros2 run ros_humans_ros2 map_publisher --ros-args -p yaml_file:=path/to/map.yaml
"""

import os
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from PIL import Image


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        
        # Declare parameters
        self.declare_parameter("yaml_file", "")
        self.declare_parameter("publish_rate", 1.0)  # Hz
        self.declare_parameter("frame_id", "map")
        
        # Get parameters
        yaml_file = self.get_parameter("yaml_file").value
        publish_rate = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value
        
        if not yaml_file:
            self.get_logger().error("No yaml_file parameter provided!")
            self.get_logger().info("Usage: ros2 run ros_humans_ros2 map_publisher --ros-args -p yaml_file:=/path/to/map.yaml")
            return
        
        yaml_file = os.path.expanduser(yaml_file)
        if not os.path.exists(yaml_file):
            self.get_logger().error(f"YAML file not found: {yaml_file}")
            return
        
        # Load map
        self.get_logger().info(f"Loading map from: {yaml_file}")
        self.map_msg = self.load_map(yaml_file)
        
        if self.map_msg is None:
            self.get_logger().error("Failed to load map!")
            return
        
        # Publish with TRANSIENT_LOCAL so late-joining subscribers
        # (like astar_path_planner) still receive the map even if they
        # start after the first publish.
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", qos)

        # Publish once immediately so the map is available right away,
        # then keep publishing on a slow timer for any new subscribers.
        self.publish_map()
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_map)
        
        self.get_logger().info(f"Publishing map at {publish_rate} Hz")
        self.get_logger().info(f"Map size: {self.map_msg.info.width}x{self.map_msg.info.height}")
        self.get_logger().info(f"Resolution: {self.map_msg.info.resolution} m/pixel")
        self.get_logger().info(f"Origin: ({self.map_msg.info.origin.position.x:.3f}, {self.map_msg.info.origin.position.y:.3f})")
    
    def load_map(self, yaml_file):
        """Load map from YAML and PGM files"""
        try:
            # Read YAML metadata
            with open(yaml_file, 'r') as f:
                map_metadata = yaml.safe_load(f)
            
            # Get PGM file path (relative to YAML file)
            yaml_dir = os.path.dirname(yaml_file)
            image_file = map_metadata.get('image', '')
            if not os.path.isabs(image_file):
                image_file = os.path.join(yaml_dir, image_file)
            
            if not os.path.exists(image_file):
                self.get_logger().error(f"Image file not found: {image_file}")
                return None
            
            # Load image
            img = Image.open(image_file)
            img_array = np.array(img)
            
            # Convert to occupancy grid values
            # PGM: 255=free(white), 0=occupied(black), 205=unknown(gray)
            # OccupancyGrid: 0=free, 100=occupied, -1=unknown
            occupancy_grid = np.full(img_array.shape, -1, dtype=np.int8)
            
            # Free space (white pixels, value > 250)
            occupancy_grid[img_array > 250] = 0
            
            # Occupied space (black pixels, value < 10)
            occupancy_grid[img_array < 10] = 100
            
            # Unknown space (gray pixels, 10-250)
            occupancy_grid[(img_array >= 10) & (img_array <= 250)] = -1
            
            # Create OccupancyGrid message
            msg = OccupancyGrid()
            msg.header.frame_id = self.frame_id
            msg.info.resolution = float(map_metadata.get('resolution', 0.05))
            msg.info.width = img_array.shape[1]
            msg.info.height = img_array.shape[0]
            
            # Origin (lower-left corner of the map)
            origin = map_metadata.get('origin', [0.0, 0.0, 0.0])
            msg.info.origin.position.x = float(origin[0])
            msg.info.origin.position.y = float(origin[1])
            msg.info.origin.position.z = 0.0
            msg.info.origin.orientation.w = 1.0
            
            # Flatten the grid (row-major order, bottom to top)
            # Note: Image rows go top-to-bottom, but map rows go bottom-to-top
            msg.data = np.flipud(occupancy_grid).flatten().tolist()
            
            self.get_logger().info("Map loaded successfully!")
            return msg
            
        except Exception as e:
            self.get_logger().error(f"Error loading map: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None
    
    def publish_map(self):
        """Publish the map with current timestamp"""
        if self.map_msg is not None:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.map_msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
