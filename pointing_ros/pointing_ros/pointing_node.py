import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from yolo_msgs.msg import DetectionArray
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
import json
from rclpy.qos import qos_profile_sensor_data

class PointingNode(Node):
    def __init__(self):
        super().__init__('pointing_node')
        self.bridge = CvBridge()
        
        # Índices de los keypoints de cada parte del cuerpo (formato COCO)
        self.L_SHOULDER = 6
        self.R_SHOULDER = 7
        self.L_ELBOW = 8
        self.R_ELBOW = 9
        self.L_WRIST = 10
        self.R_WRIST = 11

        # Suscripciones a las detecciones de YOLO (pose y detección)
        self.pose_sub = message_filters.Subscriber(self, DetectionArray, '/yolo_pose/detections', qos_profile=1)
        self.det_sub = message_filters.Subscriber(self, DetectionArray, '/yolo_detection/detections', qos_profile=1)
        
        # Se obtiene la imagen más reciente
        self.latest_image_msg = None
        self.image_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.image_callback, qos_profile=qos_profile_sensor_data)
        
        # Sincronizador aproximado (margen de 0.1s, queue_size=2 asegura solo msgs recientes)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.pose_sub, self.det_sub], queue_size=2, slop=0.1)
        self.ts.registerCallback(self.callback)
        
        # Publicadores de resultados
        self.debug_pub = self.create_publisher(Image, 'dbg_pointing', 10)
        self.detection_pub = self.create_publisher(String, 'pointing/detections', 10)
        
        self.get_logger().info('Nodo Pointing inicializado y esperando mensajes...')

    def get_keypoint(self, detection, target_id):
        if not detection.keypoints or not detection.keypoints.data:
            return None
        for kp in detection.keypoints.data:
            if kp.id == target_id: # Se busca el keypoint por su id para asegurar que es el correcto
                return (int(kp.point.x), int(kp.point.y))
        return None

    def ray_intersects_bbox(self, p1, p2, bbox):
        """
        Comprueba si el rayo que empieza en p1 y pasa por p2 interseca la BBox 2D.
        p1 y p2 son (x, y). bbox es (xmin, ymin, xmax, ymax).
        """
        x1, y1 = p1
        x2, y2 = p2
        dx = x2 - x1
        dy = y2 - y1
        
        if dx == 0 and dy == 0:
            return False
            
        xmin, ymin, xmax, ymax = bbox
        
        # Ecuación de la recta: P(t) = P1 + t*(P2-P1), t > 0
        t_values = []
        
        if dx != 0:
            t1 = (xmin - x1) / dx
            t2 = (xmax - x1) / dx
            t_values.extend([t1, t2])
        if dy != 0:
            t1 = (ymin - y1) / dy
            t2 = (ymax - y1) / dy
            t_values.extend([t1, t2])
            
        valid_t = []
        for t in t_values:
            if t > 0:
                ix = x1 + t * dx
                iy = y1 + t * dy
                if xmin <= ix <= xmax and ymin <= iy <= ymax:
                    valid_t.append(t)
        
        if valid_t:
            return True, min(valid_t)
            
        return False, None

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def callback(self, pose_msg, det_msg):
        image_msg = self.latest_image_msg
        if image_msg is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error de CV Bridge: {e}')
            return

        results = []
        
        # Extraer las bboxes de los objetos (excluyendo a las personas para evitar apuntarse a uno mismo)
        objects = []
        for det in det_msg.detections:
            if det.class_name != 'person':
                # Formato de BBox centrado a min/max
                cx = det.bbox.center.position.x
                cy = det.bbox.center.position.y
                w = det.bbox.size.x
                h = det.bbox.size.y
                bbox = (cx - w/2, cy - h/2, cx + w/2, cy + h/2)
                objects.append({'det': det, 'bbox': bbox})
                
                # Dibujar las bbox de los objetos en la imagen
                cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (255, 0, 0), 2)
                cv2.putText(cv_image, det.class_name, (int(bbox[0]), int(bbox[1]) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

        # Procesar a la persona y a dónde apunta
        for person in pose_msg.detections:
            if person.class_name != 'person':
                continue
                
            person_id = person.id if person.id else "desconocido"
            
            # Comprobar ambos brazos
            arms = [
                (self.R_ELBOW, self.R_WRIST, (0, 255, 0)), # Brazo derecho
                (self.L_ELBOW, self.L_WRIST, (0, 255, 255)) # Brazo izquierdo
            ]
            
            for elbow_idx, wrist_idx, color in arms:
                p_elbow = self.get_keypoint(person, elbow_idx)
                p_wrist = self.get_keypoint(person, wrist_idx)
                
                if p_elbow and p_wrist:
                    # Dibujar línea del brazo
                    cv2.line(cv_image, p_elbow, p_wrist, color, 3)
                    
                    # Rayo hacia donde apunta (desde la muñeca hasta el infinito)
                    direction = (p_wrist[0] - p_elbow[0], p_wrist[1] - p_elbow[1])
                    length = np.sqrt(direction[0]**2 + direction[1]**2)
                    if length < 5: continue # Demasiado corto para determinar la dirección
                    
                    # Dibujar el rayo
                    ray_end = (int(p_wrist[0] + direction[0] * 100), int(p_wrist[1] + direction[1] * 100))
                    cv2.arrowedLine(cv_image, p_wrist, ray_end, color, 2)
                    
                    # Comprobar intersección con los objetos
                    best_obj = None
                    min_t = float('inf')
                    
                    for obj in objects:
                        intersects, t = self.ray_intersects_bbox(p_elbow, p_wrist, obj['bbox'])
                        if intersects and t > 1.0: # t > 1 significa después de la muñeca
                            if t < min_t:
                                min_t = t
                                best_obj = obj
                    
                    if best_obj:
                        obj_name = best_obj['det'].class_name
                        self.get_logger().info(f'La persona {person_id} está apuntando a {obj_name}')
                        
                        # Resaltar objetivo
                        bbox = best_obj['bbox']
                        cv2.rectangle(cv_image, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), (0, 0, 255), 4)
                        
                        results.append({
                            'person_id': person_id,
                            'target_object': obj_name,
                            'target_id': best_obj['det'].id
                        })

        # Publicar resultados
        if results:
            msg = String()
            msg.data = json.dumps(results)
            self.detection_pub.publish(msg)
            
        # Publicar imagen
        try:
            image_msg_out = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            image_msg_out.header = image_msg.header
            self.debug_pub.publish(image_msg_out)
        except Exception as e:
            self.get_logger().error(f'Error al publicar en CV: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = PointingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
