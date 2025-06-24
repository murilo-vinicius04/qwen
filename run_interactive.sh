#!/bin/bash

# Script para executar Lisa Chatbot interativamente
# Com suporte completo a áudio e permissões

echo "🚀 Iniciando Lisa Chatbot - Modo Interativo"
echo "=========================================="

# Verifica se a imagem existe
if ! docker image inspect lisa_chatbot >/dev/null 2>&1; then
    echo "❌ Imagem 'lisa_chatbot' não encontrada!"
    echo "Execute primeiro: docker build -t lisa_chatbot ."
    exit 1
fi

# Para qualquer container anterior
echo "🛑 Parando containers anteriores..."
docker stop lisa_interactive 2>/dev/null || true
docker rm lisa_interactive 2>/dev/null || true

# Executa o container interativamente
echo "🎵 Iniciando container com suporte a áudio..."
echo "Pressione Ctrl+C para sair"
echo ""

docker run -it \
    --name lisa_interactive \
    --user="1000:29" \
    --group-add=audio \
    --privileged \
    --net=host \
    --device /dev/snd:/dev/snd \
    --env ROS_LOG_DIR=/tmp/ros_log \
    lisa_chatbot \
    bash

# Cleanup ao sair
echo ""
echo "🧹 Limpando container..."
docker rm lisa_interactive 2>/dev/null || true
echo "✅ Finalizado!" 