import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import mysql.connector


class DatabaseSubscriber(Node):
    def __init__(self):
        super().__init__('database_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'sensor_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Connexion à la base de données MySQL
        self.db_connection = mysql.connector.connect(
            host="localhost",
            user="root",  # Remplacez par votre utilisateur MySQL
            password="password",  # Remplacez par votre mot de passe MySQL
            database="ros2_data"
        )
        self.cursor = self.db_connection.cursor()

    def listener_callback(self, msg):
        sensor_value = msg.data
        self.get_logger().info(f'Received: {sensor_value}')

        # Insertion dans la base de données
        query = "INSERT INTO sensor_data (sensor_value) VALUES (%s)"
        self.cursor.execute(query, (sensor_value,))
        self.db_connection.commit()
        self.get_logger().info('Data saved to database')


def main(args=None):
    rclpy.init(args=args)
    node = DatabaseSubscriber()
    rclpy.spin(node)

    # Close database connection when shutting down
    node.cursor.close()
    node.db_connection.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
