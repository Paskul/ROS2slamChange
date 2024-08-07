import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class MapListenerNode(Node):

    def __init__(self):
        super().__init__('map_listener_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning

    def map_callback(self, msg):
        map_width = msg.info.width
        map_height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info('Iterating over the map data:')
        
        for i in range(map_height):
            for j in range(map_width):
                index = i * map_width + j
                occupancy_value = msg.data[index]
                x = origin_x + j * resolution
                y = origin_y + i * resolution
                self.get_logger().info(f"Coordinate: [{x:.2f}, {y:.2f}], Occupancy: {occupancy_value}")

def main(args=None):
    rclpy.init(args=args)
    node = MapListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
