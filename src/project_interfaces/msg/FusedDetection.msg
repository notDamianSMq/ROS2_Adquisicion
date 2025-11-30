# Detección fusionada con información de distancia del LiDAR
int32 class_id
string class_name
float32 confidence

# Bounding box en píxeles
float32 x1
float32 y1
float32 x2
float32 y2

# Información de distancia (del LiDAR)
float32 distance_mean      # Distancia media de los puntos dentro del bbox
float32 distance_min       # Distancia mínima
float32 distance_max       # Distancia máxima
int32 num_lidar_points     # Número de puntos LiDAR en el bbox
