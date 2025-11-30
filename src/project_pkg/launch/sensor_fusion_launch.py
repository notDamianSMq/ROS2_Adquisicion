"""
Launch file para el sistema de fusión de sensores.

Este launch file inicia todos los nodos necesarios para:
1. Procesar la nube de puntos del LiDAR
2. Detectar objetos con YOLO
3. Fusionar ambas fuentes para calcular distancias
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declarar argumentos configurables
    precision_arg = DeclareLaunchArgument(
        'precision',
        default_value='3',
        description='Precisión de limpieza de la nube de puntos (0-10)'
    )
    
    output_video_arg = DeclareLaunchArgument(
        'output_video',
        default_value='fused_output.avi',
        description='Nombre del archivo de video de salida'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Activar modo debug para visualizar proyección LiDAR'
    )
    
    # Parámetros de calibración ajustables
    camera_fx_arg = DeclareLaunchArgument(
        'camera_fx',
        default_value='1400.0',
        description='Distancia focal X de la cámara'
    )
    
    camera_fy_arg = DeclareLaunchArgument(
        'camera_fy',
        default_value='1400.0',
        description='Distancia focal Y de la cámara'
    )
    
    camera_cx_arg = DeclareLaunchArgument(
        'camera_cx',
        default_value='960.0',
        description='Centro óptico X (normalmente width/2)'
    )
    
    camera_cy_arg = DeclareLaunchArgument(
        'camera_cy',
        default_value='600.0',
        description='Centro óptico Y (normalmente height/2)'
    )
    
    return LaunchDescription([
        precision_arg,
        output_video_arg,
        debug_mode_arg,
        camera_fx_arg,
        camera_fy_arg,
        camera_cx_arg,
        camera_cy_arg,
        
        # Servidor de limpieza de nubes de puntos
        Node(
            package='project_pkg',
            executable='clean_cloud_service',
            name='clean_cloud_server',
            emulate_tty=True,
            output='screen',
        ),
        
        # Nodo que escucha el LiDAR y solicita limpieza
        Node(
            package='project_pkg',
            executable='listener_points',
            name='cloud_points_node',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'precision': LaunchConfiguration('precision')
            }]
        ),
        
        # Procesador de imagen con YOLO
        Node(
            package='project_pkg',
            executable='image_processor',
            name='image_processor',
            emulate_tty=True,
            output='screen',
        ),
        
        # Nodo de fusión de sensores
        Node(
            package='project_pkg',
            executable='sensor_fusion',
            name='sensor_fusion_node',
            emulate_tty=True,
            output='screen',
            parameters=[{
                'output_video': LaunchConfiguration('output_video'),
                'video_fps': 10,
                'debug_mode': LaunchConfiguration('debug_mode'),
                # Parámetros de calibración cámara (1920x1200)
                'camera_fx': LaunchConfiguration('camera_fx'),
                'camera_fy': LaunchConfiguration('camera_fy'),
                'camera_cx': LaunchConfiguration('camera_cx'),
                'camera_cy': LaunchConfiguration('camera_cy'),
                'image_width': 1920,
                'image_height': 1200,
                # Offset LiDAR -> Cámara
                'lidar_to_camera_x': 0.0,
                'lidar_to_camera_y': 0.0,
                'lidar_to_camera_z': 0.0,
                # Distancia del sensor al parachoques (metros)
                # Ajusta según dónde esté montado el LiDAR en tu vehículo
                'sensor_to_bumper': 2.0,  # ej: 2 metros del LiDAR al parachoques
            }]
        ),
    ])
