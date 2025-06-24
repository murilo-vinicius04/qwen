#!/usr/bin/env python3
"""
Launch file para o Lisa Chatbot ROS2
Inicia todos os nós necessários: STT, LLM e TTS
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Parâmetros de configuração
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level para os nós'
        ),
        
        # Mensagem de início
        LogInfo(msg="🤖 Iniciando Lisa Chatbot ROS2..."),
        
        # Nó STT (Speech-to-Text)
        Node(
            package='lisa_chatbot',
            executable='stt_node',
            name='stt_node',
            output='screen'
        ),
        
        # Nó LLM (Language Model)
        Node(
            package='lisa_chatbot',
            executable='llm_node',
            name='llm_node',
            output='screen'
        ),
        
        # Nó TTS (Text-to-Speech)
        Node(
            package='lisa_chatbot',
            executable='tts_node',
            name='tts_node',
            output='screen'
        ),
        
        # Mensagem de finalização
        LogInfo(msg="✅ Todos os nós do Lisa Chatbot foram iniciados!"),
        LogInfo(msg="📺 Use 'ros2 topic echo /user_question_text' para ver perguntas"),
        LogInfo(msg="📻 Use 'ros2 topic echo /lisa_response_text' para ver respostas"),
    ])
