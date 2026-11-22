"""Start the complete RO603 Week 8 social-HRI system.

Examples
  ros2 launch unitree_g1_social_hri social_hri.launch.py                       # rule-based, viewer on
  ros2 launch unitree_g1_social_hri social_hri.launch.py llm_backend:=ollama
  ros2 launch unitree_g1_social_hri social_hri.launch.py llm_backend:=openai tts_backend:=espeak
  ros2 launch unitree_g1_social_hri social_hri.launch.py input_mode:=voice vosk_model_path:=$HOME/...

Typed commands: run `ros2 run unitree_g1_social_hri speech_input_node` in a second
terminal (launch files cannot read the keyboard), or use scripts/run_text_demo.sh.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node

PKG = "unitree_g1_social_hri"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("llm_backend", default_value="rule", description="rule | ollama | openai"),
        DeclareLaunchArgument("input_mode", default_value="none", description="none | voice"),
        DeclareLaunchArgument("stt_engine", default_value="vosk", description="vosk | whisper"),
        DeclareLaunchArgument("vosk_model_path", default_value=""),
        DeclareLaunchArgument("tts_backend", default_value="print", description="print | espeak | piper | pyttsx3"),
        DeclareLaunchArgument("piper_voice", default_value=""),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("model_dir", default_value="", description="folder with g1_29dof.xml"),
        DeclareLaunchArgument("log_dir", default_value="~/ro603_hri_logs"),
    ]
    nodes = [
        Node(package=PKG, executable="g1_simulation_node", output="screen",
             parameters=[{"headless": LC("headless"), "model_dir": LC("model_dir")}]),
        Node(package=PKG, executable="action_validator_node", output="screen"),
        Node(package=PKG, executable="llm_interaction_node", output="screen",
             parameters=[{"llm_backend": LC("llm_backend")}]),
        Node(package=PKG, executable="speech_output_node", output="screen",
             parameters=[{"tts_backend": LC("tts_backend"), "piper_voice": LC("piper_voice")}]),
        Node(package=PKG, executable="hri_monitor_node", output="screen",
             parameters=[{"log_dir": LC("log_dir")}]),
        Node(package=PKG, executable="speech_input_node", output="screen",
             parameters=[{"input_mode": LC("input_mode"), "stt_engine": LC("stt_engine"),
                          "vosk_model_path": LC("vosk_model_path")}]),
    ]
    return LaunchDescription(args + nodes)
