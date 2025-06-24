#!/bin/bash

# Script para testar o sistema TTS da Lisa

echo "🎤 Testando Sistema TTS da Lisa"
echo "==============================="

# Verifica se o container está rodando
if ! docker ps | grep -q lisa_running; then
    echo "❌ Container 'lisa_running' não está rodando!"
    echo "Execute primeiro: ./run_lisa.sh"
    exit 1
fi

echo "🧠 Enviando sinal de processamento..."
docker exec lisa_running bash -c "export ROS_LOG_DIR=/tmp/ros_log && source /opt/ros/humble/setup.bash && ros2 topic pub --once /lisa_response_text std_msgs/String '{data: \"[LISA_PROCESSING]\"}'"

sleep 1

echo "💬 Enviando mensagem de teste..."
docker exec lisa_running bash -c "export ROS_LOG_DIR=/tmp/ros_log && source /opt/ros/humble/setup.bash && ros2 topic pub --once /lisa_response_text std_msgs/String '{data: \"Olá, eu sou a Lisa e estou funcionando perfeitamente com síntese de voz em tempo real.\"}'"

sleep 1

echo "✅ Finalizando processamento..."
docker exec lisa_running bash -c "export ROS_LOG_DIR=/tmp/ros_log && source /opt/ros/humble/setup.bash && ros2 topic pub --once /lisa_response_text std_msgs/String '{data: \"[LISA_FINISHED]\"}'"

echo ""
echo "🎵 Você deve ter ouvido a mensagem de teste!"
echo "Se não ouviu, verifique:"
echo "  - Volume do sistema"
echo "  - Permissões de áudio"
echo "  - docker logs lisa_running" 