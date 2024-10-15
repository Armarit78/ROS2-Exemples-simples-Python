import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudComparator(Node):
    def __init__(self):
        super().__init__('pointcloud_comparator')

        # Stocker les anciens points et les points traduits
        self.original_points = None
        self.translated_points = None

        # Souscrire au topic original
        self.subscription_original = self.create_subscription(
            PointCloud2,
            'perf/points',  # Nom du topic original
            self.original_callback,
            10
        )

        # Souscrire au topic traduit
        self.subscription_translated = self.create_subscription(
            PointCloud2,
            'translated_pointcloud',  # Nom du topic traduit
            self.translated_callback,
            10
        )

    def original_callback(self, msg):
        # Lire les points originaux
        self.original_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.compare_points()

    def translated_callback(self, msg):
        # Lire les points traduits
        self.translated_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.compare_points()

    def compare_points(self):
        # Comparer et afficher les points si les deux sets sont disponibles
        if self.original_points and self.translated_points:
            if len(self.original_points) == len(self.translated_points):
                for original, translated in zip(self.original_points, self.translated_points):
                    original_str = f"({original[0]:.2f}, {original[1]:.2f}, {original[2]:.2f})"
                    translated_str = f"({translated[0]:.2f}, {translated[1]:.2f}, {translated[2]:.2f})"
                    print(f"{original_str} -> {translated_str}")
            else:
                self.get_logger().warning("Le nombre de points dans les deux nuages de points ne correspond pas.")
        else:
            self.get_logger().info("Attente des deux nuages de points...")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudComparator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

