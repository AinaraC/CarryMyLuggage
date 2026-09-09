import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import math

class FollowNode(Node):
    """
    Nodo encargado de seguir a una persona detectada por el sensor láser.
    Se utiliza el paquete Nav2 para la navegación hacia la posición de la persona.
    """

    def __init__(self):
        super().__init__('follow_node')

        # Parámetro de distancia de seguridad
        self.declare_parameter('safety_distance', 1.0)
        self.safety_distance = self.get_parameter('safety_distance').get_parameter_value().double_value

        # Configuración de TF2 para las transformaciones de coordenadas
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Cliente de acción para la navegación
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Suscripción a los marcadores de personas detectadas
        self.marker_sub = self.create_subscription(
            MarkerArray,
            '/detected_people_markers',
            self.marker_callback,
            10
        )

    def marker_callback(self, msg):
        """
        Callback que se ejecuta al recibir marcadores de detección.
        Busca la persona más cercana y envía el objetivo de navegación.
        """
        if not msg.markers:
            return

        closest_person_map_pose = None
        min_distance = float('inf')
        best_dx = 0.0
        best_dy = 0.0

        try:
            # Posición actual del robot en el mapa
            robot_transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            robot_x = robot_transform.transform.translation.x
            robot_y = robot_transform.transform.translation.y

            # Recorrer todos los marcadores para encontrar el más cercano
            for marker in msg.markers:
                person_pose = PoseStamped()
                person_pose.header = marker.header
                person_pose.pose = marker.pose

                try:
                    # Transformación de la posición de esta persona al mapa
                    transform = self.tf_buffer.lookup_transform(
                        'map',
                        person_pose.header.frame_id,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    person_map_pose = do_transform_pose(person_pose.pose, transform)

                    # Calculamos distancia al robot
                    dx = person_map_pose.position.x - robot_x
                    dy = person_map_pose.position.y - robot_y
                    distance = math.sqrt(dx**2 + dy**2)

                    # Si es la más cercana hasta ahora, la guardamos
                    if distance < min_distance:
                        min_distance = distance
                        closest_person_map_pose = person_map_pose
                        best_dx = dx
                        best_dy = dy

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

            if closest_person_map_pose is not None:
                if min_distance > self.safety_distance:
                    self.get_logger().info(f'Siguiendo a la persona más cercana a {min_distance:.2f}m.')
                    self.send_navigation_goal(closest_person_map_pose, best_dx, best_dy, min_distance)
                else:
                    self.get_logger().info('Persona más cercana dentro del radio de seguridad. Robot detenido', once=True)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Error de transformación TF2: {e}')

    def send_navigation_goal(self, person_map_pose, dx, dy, distance):
        """
        Envía un objetivo de navegación a Nav2 manteniendo una distancia de seguridad.
        """
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Servidor de acción navigate_to_pose no disponible.')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Se calcula el punto de destino a 'safety_distance' metros de la persona
        # siguiendo la línea que une al robot con la persona
        goal_msg.pose.pose.position.x = person_map_pose.position.x - (dx / distance) * self.safety_distance
        goal_msg.pose.pose.position.y = person_map_pose.position.y - (dy / distance) * self.safety_distance
        goal_msg.pose.pose.position.z = 0.0

        # Orientación hacia la persona
        angle = math.atan2(dy, dx)
        goal_msg.pose.pose.orientation.z = math.sin(angle / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(angle / 2.0)

        self.nav_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = None
    try:
        node = FollowNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
