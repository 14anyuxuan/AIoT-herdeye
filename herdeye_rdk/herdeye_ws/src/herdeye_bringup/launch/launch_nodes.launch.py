from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='herdeye_sht30',
            executable='herdeye_sht30',
            name='sht30_sensor',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='herdeye_home',
            executable='herdeye_home',
            name='home_node',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='herdeye_gps',
            executable='herdeye_gps',
            name='gps_sensor',
            output='screen',
            emulate_tty=True,
        ),
        
        Node(
            package='herdeye_iot',
            executable='herdeye_iot',
            name='iot_bridge',
            output='screen',
            emulate_tty=True,
        )£¬
        Node(
            package='herdeye_yolo',
            executable='herdeye_floor_yolo',
            name='floor_yolo_detector',
            output='screen',
            emulate_tty=True,

        ),
        
        Node(
            package='herdeye_yolo',
            executable='herdeye_floor_yolo_ret',
            name='floor_yolo_retriever',
            output='screen',
            emulate_tty=True,
        )
    ])