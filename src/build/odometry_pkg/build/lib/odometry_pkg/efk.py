# ekf.py

import numpy as np
from rclpy.logging import get_logger

logger = get_logger('ekf_utils')

def compute_F(x, u, imu_data, dt):
    """
    Calcula la matriz jacobiana F para el filtro de Kalman extendido,
    usando valores seguros por defecto si faltan claves.
    """
    v = u[0]
    theta = x[2]
    a_x = imu_data.get('linear_acceleration', {}).get('x', 0.0)
    a_y = imu_data.get('angular_velocity', {}).get('y', 0.0)

    F = np.array([
        [1, 0, -v * np.sin(theta) * dt, dt, 0, 0],
        [0, 1,  v * np.cos(theta) * dt, 0, dt, 0],
        [0, 0, 1,                       0, 0, dt],
        [0, 0, -a_x * dt * np.sin(theta), 1, 0, 0],
        [0, 0,  a_y * dt * np.cos(theta), 0, 1, 0],
        [0, 0, 0,                       0, 0, 1]
    ])
    return F

def predict_state(x, u, imu_data, L, dt):
    """
    Predice el siguiente estado del sistema usando la función de transición,
    usando valores seguros por defecto si faltan claves.
    """
    v = u[0]
    theta = x[2]
    a_x = imu_data.get('linear_acceleration', {}).get('x', 0.0)
    a_y = imu_data.get('linear_acceleration', {}).get('y', 0.0)
    omega = imu_data.get('angular_velocity', {}).get('z', 0.0)

    x_next = np.array([
        x[0] + x[3] * dt,
        x[1] + x[4] * dt,
        theta + omega * dt,
        v + a_x * dt * np.cos(theta),
        v + a_y * dt * np.sin(theta),
        omega
    ])
    return x_next

