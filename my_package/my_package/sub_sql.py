import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import mysql.connector

class DataExtractor(Node):
    def __init__(self):
        super().__init__('data_extractor')
        self.publisher_ = self.create_publisher(String, 'extracted_data_topic', 10)
        self.timer = self.create_timer(5.0, self.extract_data)

        # Connexion à la base de données MySQL
        self.db_connection = mysql.connector.connect(
            host="localhost",
            user="root",
            password="password",
            database="ros2_data"
        )
        self.cursor = self.db_connection.cursor()

    def extract_data(self):
        query = "SELECT * FROM sensor_data ORDER BY timestamp DESC LIMIT 1"
        self.cursor.execute(query)
        result = self.cursor.fetchone()

        if result:
            message = String()
            message.data = f"ID: {result[0]}, Value: {result[1]}, Timestamp: {result[2]}"
            self.publisher_.publish(message)
            self.get_logger().info(f'Publishing extracted data: {message.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DataExtractor()
    rclpy.spin(node)
    
    node.cursor.close()
    node.db_connection.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
