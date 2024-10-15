import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WordChecker(Node):
    def __init__(self):
        super().__init__('word_checker')
        
        # Subscriber pour écouter le topic "word_topic"
        self.subscription = self.create_subscription(
            String,
            'word_topic',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Publisher pour envoyer des réponses
        self.publisher_ = self.create_publisher(String, 'response_topic', 10)

    def listener_callback(self, msg):
        received_word = msg.data
        response = String()

        # Vérification du mot "chat"
        if received_word == "chat":
            self.get_logger().info('Received expected word: "chat"')
            response.data = 'Oui, le message reçu est "chat".'
        else:
            self.get_logger().info(f'Received word: "{received_word}" (not expected)')
            response.data = f'Non, le message reçu est "{received_word}".'

        # Publier la réponse sur le topic "response_topic"
        self.publisher_.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = WordChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

