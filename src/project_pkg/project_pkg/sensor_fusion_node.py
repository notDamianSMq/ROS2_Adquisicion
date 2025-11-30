#!/usr/bin/env python3
"""
Sensor Fusion Node: Combina datos del LiDAR limpio con detecciones YOLO.

Este nodo:
1. Sincroniza datos de la nube de puntos limpia (/clean/points) con las detecciones YOLO (/image/predictions)
2. Para cada bounding box, proyecta los puntos LiDAR y calcula la distancia media
3. Genera un video con las bounding boxes anotadas con la distancia al obstáculo
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

from project_interfaces.msg import CleanedCloud, FusedDetection, FusedDetections, Detections


class SensorFusionNode(Node):
    def __init__(self):
        super().__init__(
            'sensor_fusion_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        self.bridge = CvBridge()
        
        # ============================================================
        # PARÁMETROS DE CALIBRACIÓN CÁMARA-LIDAR
        # ============================================================
        # Estos valores deben ajustarse según tu setup específico
        # 
        # Matriz intrínseca de la cámara (K) - 3x3
        # [fx,  0, cx]
        # [ 0, fy, cy]
        # [ 0,  0,  1]
        self.declare_parameter('camera_fx', 600.0)  # Distancia focal X
        self.declare_parameter('camera_fy', 600.0)  # Distancia focal Y
        self.declare_parameter('camera_cx', 320.0)  # Centro óptico X (width/2)
        self.declare_parameter('camera_cy', 240.0)  # Centro óptico Y (height/2)
        
        # Dimensiones de la imagen
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Transformación extrínseca LiDAR -> Cámara (simplificada)
        # Asumimos que el LiDAR y la cámara están aproximadamente alineados
        # con un offset de traslación
        self.declare_parameter('lidar_to_camera_x', 0.0)  # metros
        self.declare_parameter('lidar_to_camera_y', 0.0)  # metros
        self.declare_parameter('lidar_to_camera_z', 0.0)  # metros
        
        # Distancia desde el sensor hasta el parachoques/borde del vehículo
        # Se resta de la medición para obtener distancia real al obstáculo
        self.declare_parameter('sensor_to_bumper', 0.0)  # metros
        
        # Parámetros del video de salida
        self.declare_parameter('output_video', 'fused_output.avi')
        self.declare_parameter('video_fps', 10)
        
        # Parámetro de debug para visualizar proyección LiDAR y crops
        self.declare_parameter('debug_mode', False)


        
        # Cargar parámetros
        self._load_parameters()
        
        # ============================================================
        # SUSCRIPTORES
        # ============================================================
        # Suscriptor para la nube de puntos limpia
        self.cloud_sub = self.create_subscription(
            CleanedCloud,
            '/clean/points',
            self.cloud_callback,
            10
        )
        
        # Suscriptor para las detecciones YOLO
        self.detections_sub = self.create_subscription(
            Detections,
            '/image/predictions',
            self.detections_callback,
            10
        )
        
        # Suscriptor para la imagen raw (para visualización)
        self.image_sub = self.create_subscription(
            Image,
            '/my_camera/pylon_ros2_camera_node/image_raw',
            self.image_callback,
            10
        )
        
        # ============================================================
        # PUBLICADOR
        # ============================================================
        self.fused_publisher = self.create_publisher(
            FusedDetections,
            '/fused/detections',
            10
        )
        
        # ============================================================
        # BUFFERS PARA SINCRONIZACIÓN TEMPORAL
        # ============================================================
        self.latest_cloud = None
        self.latest_cloud_points = None  # Puntos ya procesados
        self.latest_detections = None
        self.latest_image = None
        
        # ============================================================
        # VIDEO WRITER
        # ============================================================
        self.video_writer = cv2.VideoWriter(
            self.output_video,
            cv2.VideoWriter_fourcc(*'XVID'),
            self.video_fps,
            (self.image_width, self.image_height)
        )
        
        # Colores para las bounding boxes (BGR)
        self.colors = [
            (0, 255, 0),    # Verde
            (255, 0, 0),    # Azul
            (0, 0, 255),    # Rojo
            (255, 255, 0),  # Cyan
            (255, 0, 255),  # Magenta
            (0, 255, 255),  # Amarillo
            (128, 0, 128),  # Púrpura
            (0, 128, 128),  # Teal
        ]
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Sensor Fusion Node iniciado')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Escuchando:')
        self.get_logger().info(f'    - /clean/points (LiDAR limpio)')
        self.get_logger().info(f'    - /image/predictions (YOLO)')
        self.get_logger().info(f'    - /my_camera/.../image_raw (Imagen)')
        self.get_logger().info(f'  Publicando:')
        self.get_logger().info(f'    - /fused/detections')
        self.get_logger().info(f'  Video de salida: {self.output_video}')
        self.get_logger().info(f'  Debug mode: {self.debug_mode}')
        self.get_logger().info('=' * 60)

    def _load_parameters(self):
        """Carga los parámetros de configuración."""
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.cx = self.get_parameter('camera_cx').value
        self.cy = self.get_parameter('camera_cy').value
        
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        
        self.lidar_offset = np.array([
            self.get_parameter('lidar_to_camera_x').value,
            self.get_parameter('lidar_to_camera_y').value,
            self.get_parameter('lidar_to_camera_z').value
        ])
        
        # Distancia del sensor al borde del vehículo (se resta de la medición)
        self.sensor_to_bumper = self.get_parameter('sensor_to_bumper').value
        
        self.output_video = self.get_parameter('output_video').value
        self.video_fps = self.get_parameter('video_fps').value
        
        # Manejar debug_mode que puede llegar como string o bool
        debug_val = self.get_parameter('debug_mode').value
        if isinstance(debug_val, bool):
            self.debug_mode = debug_val
        else:
            self.debug_mode = str(debug_val).lower() in ('true', '1', 'yes')
        
        # Construir matriz intrínseca
        self.K = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

    def cloud_callback(self, msg: CleanedCloud):
        """Callback para la nube de puntos limpia."""
        self.latest_cloud = msg
        
        # Pre-procesar los puntos para proyección
        points = pc2.read_points(msg.points, field_names=["x", "y", "z"], skip_nans=True)
        if points.size > 0:
            self.latest_cloud_points = np.array(points.tolist())
            self.get_logger().info(f'[CLOUD] Recibida nube con {len(self.latest_cloud_points)} puntos')
        else:
            self.latest_cloud_points = None
            self.get_logger().warn('[CLOUD] Nube vacía recibida')
        
        # Intentar fusión
        self._try_fusion()

    def detections_callback(self, msg: Detections):
        """Callback para las detecciones YOLO."""
        self.latest_detections = msg
        self.get_logger().info(f'[DETECTIONS] Recibidas {msg.num_detections} detecciones')
        
        # Intentar fusión
        self._try_fusion()

    def image_callback(self, msg: Image):
        """Callback para la imagen raw."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')

    def _try_fusion(self):
        """Intenta fusionar los datos si están disponibles."""
        if self.latest_cloud_points is None:
            self.get_logger().info('[FUSION] Esperando nube de puntos...')
            return
        if self.latest_detections is None:
            self.get_logger().info('[FUSION] Esperando detecciones...')
            return
        
        self.get_logger().info(f'[FUSION] Fusionando {len(self.latest_cloud_points)} puntos con {self.latest_detections.num_detections} detecciones')
        
        # Proyectar puntos LiDAR a coordenadas de imagen
        projected_points = self._project_lidar_to_image(self.latest_cloud_points)
        
        # Crear mensaje de detecciones fusionadas
        fused_msg = FusedDetections()
        fused_msg.header = Header()
        fused_msg.header.stamp = self.get_clock().now().to_msg()
        fused_msg.header.frame_id = 'camera_frame'
        
        fused_detections = []
        debug_info = []  # Para visualización de debug
        
        for det in self.latest_detections.detections:
            # Encontrar puntos dentro del bounding box
            bbox = (det.x1, det.y1, det.x2, det.y2)
            points_in_bbox, distances = self._get_points_in_bbox(
                projected_points, 
                self.latest_cloud_points,
                bbox
            )
            
            # Guardar info para debug
            debug_info.append({
                'bbox': bbox,
                'class_name': det.class_name,
                'points_in_bbox': points_in_bbox,
                'distances': distances
            })
            
            # Crear detección fusionada
            fused_det = FusedDetection()
            fused_det.class_id = det.class_id
            fused_det.class_name = det.class_name
            fused_det.confidence = det.confidence
            fused_det.x1 = det.x1
            fused_det.y1 = det.y1
            fused_det.x2 = det.x2
            fused_det.y2 = det.y2
            
            if len(distances) > 0:
                # Usar mediana y restar distancia del sensor al borde del vehículo
                raw_distance = float(np.median(distances))
                fused_det.distance_mean = max(0.0, raw_distance - self.sensor_to_bumper)
                fused_det.distance_min = max(0.0, float(np.min(distances)) - self.sensor_to_bumper)
                fused_det.distance_max = max(0.0, float(np.max(distances)) - self.sensor_to_bumper)
                fused_det.num_lidar_points = len(distances)
            else:
                fused_det.distance_mean = -1.0  # Indica que no hay datos
                fused_det.distance_min = -1.0
                fused_det.distance_max = -1.0
                fused_det.num_lidar_points = 0
            
            fused_detections.append(fused_det)
        
        fused_msg.detections = fused_detections
        fused_msg.num_detections = len(fused_detections)
        
        # Publicar
        self.fused_publisher.publish(fused_msg)
        
        # Visualizar y guardar frame
        self._visualize_and_save(fused_detections, projected_points, debug_info)
        
        # Log
        self._log_detections(fused_detections)
        
        # Limpiar para evitar re-procesamiento
        self.latest_detections = None

    def _project_lidar_to_image(self, points_3d: np.ndarray) -> np.ndarray:
        """
        Proyecta puntos 3D del LiDAR a coordenadas 2D de imagen.
        
        Asume un modelo de cámara pinhole simple.
        La proyección es: [u, v, 1]^T = K * [X, Y, Z]^T / Z
        
        Donde:
        - X: eje lateral (derecha positivo)
        - Y: eje vertical (abajo positivo en imagen)
        - Z: eje de profundidad (frente positivo)
        
        NOTA: Puede que necesites ajustar los ejes según tu configuración.
        """
        if len(points_3d) == 0:
            return np.array([])
        
        # Log de diagnóstico
        if self.debug_mode:
            self.get_logger().info(f'[PROJ] Puntos entrada: {len(points_3d)}')
            self.get_logger().info(f'[PROJ] Rango X: [{points_3d[:,0].min():.1f}, {points_3d[:,0].max():.1f}]')
            self.get_logger().info(f'[PROJ] Rango Y: [{points_3d[:,1].min():.1f}, {points_3d[:,1].max():.1f}]')
            self.get_logger().info(f'[PROJ] Rango Z: [{points_3d[:,2].min():.1f}, {points_3d[:,2].max():.1f}]')
        
        # Aplicar offset de calibración
        points_cam = points_3d + self.lidar_offset
        
        # Transformación de coordenadas LiDAR a cámara
        # LiDAR Ouster mirando hacia atrás: X=atrás, Y=derecha, Z=arriba
        # Cámara típica: X=derecha, Y=abajo, Z=adelante
        # Invertimos X del LiDAR porque mira en sentido contrario
        x_cam = points_cam[:, 1]   # Y_lidar -> X_cam
        y_cam = -points_cam[:, 2]  # Z_lidar -> -Y_cam
        z_cam = -points_cam[:, 0]  # -X_lidar -> Z_cam (invertido)
        
        # Filtrar puntos detrás de la cámara
        valid_mask = z_cam > 0.1  # Al menos 10cm delante
        
        if not np.any(valid_mask):
            return np.array([])
        
        x_cam = x_cam[valid_mask]
        y_cam = y_cam[valid_mask]
        z_cam = z_cam[valid_mask]
        
        # Proyección perspectiva
        u = (self.fx * x_cam / z_cam) + self.cx
        v = (self.fy * y_cam / z_cam) + self.cy
        
        # Log de diagnóstico post-proyección
        if self.debug_mode:
            self.get_logger().info(f'[PROJ] Puntos válidos (z>0.1): {len(u)}')
            self.get_logger().info(f'[PROJ] Rango U: [{u.min():.0f}, {u.max():.0f}] (imagen: 0-{self.image_width})')
            self.get_logger().info(f'[PROJ] Rango V: [{v.min():.0f}, {v.max():.0f}] (imagen: 0-{self.image_height})')
            # Cuántos caen dentro de la imagen
            in_image = ((u >= 0) & (u < self.image_width) & (v >= 0) & (v < self.image_height))
            self.get_logger().info(f'[PROJ] Puntos dentro de imagen: {np.sum(in_image)}')
        
        # Guardar también el índice original para recuperar la profundidad
        projected = np.column_stack([u, v, z_cam, np.where(valid_mask)[0]])
        
        return projected

    def _get_points_in_bbox(self, projected_points: np.ndarray, 
                            original_points: np.ndarray,
                            bbox: tuple) -> tuple:
        """
        Obtiene los puntos que caen dentro de un bounding box.
        
        Returns:
            tuple: (puntos_2d, distancias_3d)
        """
        if len(projected_points) == 0:
            return np.array([]), np.array([])
        
        x1, y1, x2, y2 = bbox
        
        # Filtrar puntos dentro del bbox
        u = projected_points[:, 0]
        v = projected_points[:, 1]
        z = projected_points[:, 2]  # Profundidad
        
        mask = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        
        points_in_bbox = projected_points[mask]
        
        if len(points_in_bbox) == 0:
            return np.array([]), np.array([])
        
        # Las distancias son simplemente la coordenada Z (profundidad)
        # Para distancia euclidiana real, usar:
        # distances = np.sqrt(x^2 + y^2 + z^2)
        distances = points_in_bbox[:, 2]
        
        return points_in_bbox[:, :2], distances

    def _visualize_and_save(self, fused_detections: list, projected_points: np.ndarray = None, debug_info: list = None):
        """Visualiza las detecciones y guarda el frame."""
        try:
            # Crear imagen base
            if self.latest_image is not None:
                frame = self.latest_image.copy()
                # Actualizar dimensiones si la imagen real es diferente
                actual_height, actual_width = frame.shape[:2]
                if actual_width != self.image_width or actual_height != self.image_height:
                    self.image_width = actual_width
                    self.image_height = actual_height
            else:
                # Si no hay imagen, crear una en negro
                frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
            
            # ============================================================
            # Dibujar bounding boxes con distancias en la imagen de la cámara
            # ============================================================
            for i, det in enumerate(fused_detections):
                color = self.colors[i % len(self.colors)]
                
                # Coordenadas del bbox
                x1, y1 = int(det.x1), int(det.y1)
                x2, y2 = int(det.x2), int(det.y2)
                
                # Dibujar rectángulo
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                
                # Preparar texto
                if det.distance_mean > 0:
                    label = f'{det.class_name}: {det.distance_mean:.1f}m'
                    sublabel = f'({det.num_lidar_points} pts)'
                else:
                    label = f'{det.class_name}'
                    sublabel = '(sin datos LiDAR)'
                
                # Fondo para el texto
                (w1, h1), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                (w2, h2), _ = cv2.getTextSize(sublabel, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                
                cv2.rectangle(frame, (x1, y1 - h1 - h2 - 10), (x1 + max(w1, w2) + 5, y1), color, -1)
                
                # Texto
                cv2.putText(frame, label, (x1 + 2, y1 - h2 - 5), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(frame, sublabel, (x1 + 2, y1 - 3), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            # ============================================================
            # DEBUG: Crear imagen del LiDAR proyectado (solo si debug_mode=True)
            # ============================================================
            if self.debug_mode and projected_points is not None and len(projected_points) > 0:
                # Crear imagen del LiDAR proyectado (mismo tamaño que la cámara)
                lidar_frame = np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8)
                
                # Dibujar TODOS los puntos LiDAR proyectados con color basado en distancia
                for p in projected_points:
                    u, v = int(p[0]), int(p[1])
                    if 0 <= u < self.image_width and 0 <= v < self.image_height:
                        # Color basado en distancia (más cerca = más rojo, más lejos = más azul)
                        dist = p[2]
                        # Normalizar distancia (asumiendo rango 0-50m)
                        norm_dist = np.clip(dist / 50.0, 0, 1)
                        # Colormap: cerca=rojo, lejos=azul
                        r = int(255 * (1 - norm_dist))
                        b = int(255 * norm_dist)
                        g = 50
                        lidar_frame[v, u] = (b, g, r)  # BGR
                
                # Dibujar los puntos dentro de cada bbox en color brillante
                if debug_info:
                    for i, info in enumerate(debug_info):
                        bbox = info['bbox']
                        points_in = info['points_in_bbox']
                        distances = info['distances']
                        class_name = info['class_name']
                        color = self.colors[i % len(self.colors)]
                        
                        x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])
                        
                        # Dibujar bbox en la imagen del LiDAR
                        cv2.rectangle(lidar_frame, (x1, y1), (x2, y2), color, 2)
                        
                        # Dibujar los puntos dentro del bbox en color brillante
                        if len(points_in) > 0:
                            for j, p in enumerate(points_in):
                                u, v = int(p[0]), int(p[1])
                                if 0 <= u < self.image_width and 0 <= v < self.image_height:
                                    cv2.circle(lidar_frame, (u, v), 3, color, -1)
                            
                            dist_mean = np.mean(distances)
                            label = f'{class_name}: {dist_mean:.1f}m ({len(distances)} pts)'
                            cv2.putText(lidar_frame, label, (x1, y1 - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        else:
                            cv2.putText(lidar_frame, f'{class_name}: SIN PUNTOS', (x1, y1 - 5),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # Mostrar ventana de LiDAR (solo en debug)
                cv2.imshow('LiDAR Proyectado', lidar_frame)
            
            # Mostrar ventana de la cámara (siempre)
            cv2.imshow('Sensor Fusion', frame)
            cv2.waitKey(1)
            
            # Guardar en video
            self.video_writer.write(frame)
            
        except Exception as e:
            self.get_logger().error(f'[ERROR] Error en visualización: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def _log_detections(self, fused_detections: list):
        """Log de las detecciones fusionadas."""
        if len(fused_detections) == 0:
            return
        
        self.get_logger().info(f'--- Detecciones Fusionadas: {len(fused_detections)} ---')
        for det in fused_detections:
            if det.distance_mean > 0:
                self.get_logger().info(
                    f'  {det.class_name}: {det.distance_mean:.2f}m '
                    f'(min={det.distance_min:.2f}m, max={det.distance_max:.2f}m, '
                    f'{det.num_lidar_points} puntos)'
                )
            else:
                self.get_logger().info(f'  {det.class_name}: sin datos de distancia')

    def destroy_node(self):
        """Limpieza al cerrar el nodo."""
        if self.video_writer is not None:
            self.video_writer.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Nodo interrumpido por el usuario')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
