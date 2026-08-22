# Обнаруженный объект компьютерного зрения (YOLO / детектор H-маркера).
# Публикуется нодами uav_perception в топик /detections.

std_msgs/Header header    # Временная метка и frame_id исходного кадра

string  class_name        # Имя класса: "person", "h_marker", ...
float32 confidence        # Уверенность детекции [0.0 .. 1.0]

# Ограничивающая рамка в пикселях исходного изображения
uint32  x_min
uint32  y_min
uint32  x_max
uint32  y_max
