#!/bin/bash

# Script para executar Lisa Chatbot em background
# Com suporte completo a áudio e permissões

echo "🤖 Iniciando Lisa Chatbot - Modo Background"
echo "==========================================="

# Verifica se a imagem existe
if ! docker image inspect lisa_chatbot >/dev/null 2>&1; then
    echo "❌ Imagem 'lisa_chatbot' não encontrada!"
    echo "Execute primeiro: docker build -t lisa_chatbot ."
    exit 1
fi

# Para qualquer container anterior
echo "🛑 Parando containers anteriores..."
docker stop lisa_running 2>/dev/null || true
docker rm lisa_running 2>/dev/null || true

# Executa o container em background
echo "🎵 Iniciando Lisa em background com suporte a áudio..."

docker run -d \
    --name lisa_running \
    --user="1000:29" \
    --group-add=audio \
    --privileged \
    --net=host \
    --device /dev/snd:/dev/snd \
    --env ROS_LOG_DIR=/tmp/ros_log \
    lisa_chatbot

if [ $? -eq 0 ]; then
    echo "✅ Lisa Chatbot iniciada com sucesso!"
    echo ""
    echo "📋 Comandos úteis:"
    echo "  docker logs lisa_running          # Ver logs"
    echo "  docker exec -it lisa_running bash # Entrar no container"
    echo "  docker stop lisa_running          # Parar a Lisa"
    echo ""
    echo "🎯 Para testar o TTS, execute:"
    echo "  ./test_tts.sh"
else
    echo "❌ Erro ao iniciar o container!"
    exit 1
fi 