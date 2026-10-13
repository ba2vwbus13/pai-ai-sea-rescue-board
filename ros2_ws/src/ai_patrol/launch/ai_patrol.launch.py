"""AI 海難救助システム(ドローン + フォイルボード)の ROS 2 ノード一式を起動する。

前提: PX4 SITL + Gazebo(scripts/start_sim.sh)と MicroXRCEAgent が起動していること。
  ros2 launch ai_patrol ai_patrol.launch.py model:=<GPU用onnx> cpu_model:=<CPU用onnx>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

WORLD = 'sea_board'
CAMERA_TOPIC = f'/world/{WORLD}/model/x500_sar_cam_0/link/camera_link/sensor/camera/image'


def generate_launch_description():
    params = os.path.join(get_package_share_directory('ai_patrol'), 'config', 'params.yaml')
    model = LaunchConfiguration('model')
    cpu_model = LaunchConfiguration('cpu_model')
    report_dir = LaunchConfiguration('report_dir')
    dash = {k: LaunchConfiguration(k) for k in ('fullscreen', 'window_x', 'window_y')}
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=''),
        DeclareLaunchArgument('cpu_model', default_value=''),
        DeclareLaunchArgument('report_dir', default_value=os.path.expanduser('~/work/pai_board/results')),
        # 表示画面を録画用モニタに全画面表示する場合: fullscreen:=true window_x:=2048 window_y:=72
        DeclareLaunchArgument('fullscreen', default_value='false'),
        DeclareLaunchArgument('window_x', default_value='-1'),
        DeclareLaunchArgument('window_y', default_value='-1'),
        # Gazebo のカメラ画像を ROS 2 へ
        Node(package='ros_gz_image', executable='image_bridge', arguments=[CAMERA_TOPIC],
             output='log'),
        Node(package='ai_patrol', executable='detector', output='screen',
             parameters=[params, {'image_topic': CAMERA_TOPIC, 'model_path': model,
                                  'cpu_model_path': cpu_model}]),
        Node(package='ai_patrol', executable='mission', output='screen',
             parameters=[params, {'report_dir': report_dir}]),
        # フォイルボードのシミュレーション。gz-transport の Python を使うので、AI 用仮想環境(新しい
        # protobuf を含む)を PYTHONPATH から外して起動する
        Node(package='ai_patrol', executable='board', output='screen',
             parameters=[params, {'world': WORLD}],
             additional_env={'PYTHONPATH': ':'.join(
                 p for p in os.environ.get('PYTHONPATH', '').split(':') if 'pai_venv' not in p)}),
        # ドローン → ボートの LoRa 無線リンクの模擬
        Node(package='ai_patrol', executable='lora_link', output='log', parameters=[params]),
        Node(package='ai_patrol', executable='dashboard', output='log',
             parameters=[{'report_dir': report_dir,
                          'fullscreen': ParameterValue(dash['fullscreen'], value_type=bool),
                          'window_x': ParameterValue(dash['window_x'], value_type=int),
                          'window_y': ParameterValue(dash['window_y'], value_type=int)}]),
    ])
