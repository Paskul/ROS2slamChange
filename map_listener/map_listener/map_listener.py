import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
# LIKELY HERE NEED TO ADD NEW MSG,
# EITHER FROM NEW MSG MADE BY USER, OR ALREADY USED MSG ELSEWHERE

class MapListenerNode(Node):

    def __init__(self):
        super().__init__('map_listener_node')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10)
        # prevent unused variable warning
        self.subscription

        # as well as this, subscribe to any new topic that generates the [0,1] occupancy of cells in grid

    def map_callback(self, msg):
        map_width = msg.info.width
        map_height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        self.get_logger().info('Iterating over the map data:')

        # can calculate seperatly to index by exact cell's and their resolution
        # truthfully, index is prefered for now to show that a cell's occupancy lines up with it's static state value
        
        # by integer index
        for i in range(map_height):
            for j in range(map_width):
                index = i * map_width + j
                occupancy_value = msg.data[index]
                self.get_logger().info(f"Grid Index: [{i}, {j}], Occupancy: {occupancy_value}")

        # by cord decimal
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
