import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import struct
import numpy as np

class PointCloudTranslator(Node):
    def __init__(self):
        super().__init__('pointcloud_translator')

        # Souscrire au topic de pointcloud d'origine
        self.subscription = self.create_subscription(
            PointCloud2,
            'perf/points',  # Nom du topic à souscrire
            self.pointcloud_callback,
            10
        )

        # Publisher pour publier le nuage de points modifié
        self.publisher_ = self.create_publisher(PointCloud2, 'translated_pointcloud', 10)

    def pointcloud_callback(self, msg):
        try:
            # Vérification que les champs x, y, z existent
            if not all(f in [field.name for field in msg.fields] for f in ['x', 'y', 'z']):
                self.get_logger().error("Le message PointCloud2 ne contient pas les champs 'x', 'y' et 'z'")
                return

            # Récupérer les données binaires
            binary_data = msg.data

            # Initialiser une liste pour les points traduits
            translated_points = []

            # Parcourir les données binaires en incréments de point_step (12 octets : 3 * 4 octets pour x, y, z)
            for i in range(0, len(binary_data), msg.point_step):
                # Extraire les 12 octets pour x, y, z
                x, y, z = struct.unpack_from('fff', binary_data, offset=i)
                
                # Appliquer la translation de +2 en décimal
                x_translated = x + 2.0
                y_translated = y + 2.0
                z_translated = z + 2.0

                # Reconvertir en binaire après translation
                translated_points.append(struct.pack('fff', x_translated, y_translated, z_translated))

            # Combiner les points traduits en un seul bloc de données binaires
            translated_binary_data = b''.join(translated_points)

            # Créer un nouveau message PointCloud2 avec les points traduits
            translated_msg = PointCloud2()
            translated_msg.header = msg.header  # Réutiliser le même header
            translated_msg.height = msg.height
            translated_msg.width = msg.width
            translated_msg.fields = msg.fields  # Réutiliser les mêmes champs
            translated_msg.is_bigendian = msg.is_bigendian
            translated_msg.point_step = msg.point_step
            translated_msg.row_step = msg.row_step
            translated_msg.is_dense = msg.is_dense
            translated_msg.data = translated_binary_data  # Nouveaux points traduits

            # Publier le message traduit
            self.publisher_.publish(translated_msg)
            self.get_logger().info('Nuage de points traduit et publié.')

        except Exception as e:
            self.get_logger().error(f"Erreur lors de la traduction du nuage de points : {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

