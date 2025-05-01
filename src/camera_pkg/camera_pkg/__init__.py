# camera_pkg/__init__.py

"""
camera_pkg: rock-detection nodes for ROS 2 Humble using YOLOv8.
"""

__version__ = '0.1.0'

# expose your main entry point directly at package level:
from .framepublisher import main
