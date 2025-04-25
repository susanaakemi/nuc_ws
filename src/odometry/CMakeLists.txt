cmake_minimum_required(VERSION 3.5)

project(odometry)  # Reemplaza con el nombre de tu paquete

# Encuentra el paquete de ament_cmake, que es necesario para ROS 2
find_package(ament_cmake REQUIRED)

# Encontrar rclpy, que es necesario para los scripts de Python
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
# Si tienes más dependencias de Python, agrégalas aquí

# Instalar los scripts de Python
install(PROGRAMS
  scripts/controller.py
  scripts/efk.py
  scripts/odometry_node.py
  scripts/utils.py  # Asegúrate de agregar todos los scripts que quieras instalar
  DESTINATION lib/${PROJECT_NAME}  # Los instalará en la carpeta lib/mi_paquete
)

ament_package()  # Esto es necesario para finalizar la configuración del paquete
