#!/bin/bash

# Script para executar Lisa Chatbot em modo desenvolvimento
# Com volume mount para ver arquivos de build localmente

echo "🛠️ Iniciando Lisa Chatbot - Modo Desenvolvimento"
echo "==============================================="

# Verifica se a imagem existe
if ! docker image inspect lisa_chatbot >/dev/null 2>&1; then
    echo "❌ Imagem 'lisa_chatbot' não encontrada!"
    echo "Execute primeiro: docker build -t lisa_chatbot ."
    exit 1
fi

# Para qualquer container anterior
echo "🛑 Parando containers anteriores..."
docker stop lisa_dev 2>/dev/null || true
docker rm lisa_dev 2>/dev/null || true

# Executa o container com volume mount
echo "🎵 Iniciando Lisa em modo desenvolvimento..."
echo "📁 Workspace será montado em: $(pwd)"

docker run -d \
    --name lisa_dev \
    --user="1000:29" \
    --group-add=audio \
    --privileged \
    --net=host \
    --device /dev/snd:/dev/snd \
    --env ROS_LOG_DIR=/tmp/ros_log \
    -v $(pwd):/workspace \
    -w /workspace \
    lisa_chatbot \
    bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install && source install/setup.bash && ros2 launch lisa_chatbot lisa.launch.py"

if [ $? -eq 0 ]; then
    echo "✅ Lisa Chatbot iniciada em modo desenvolvimento!"
    echo ""
    echo "📁 Agora você verá as pastas build/, install/, log/ localmente!"
    echo ""
    echo "📋 Comandos úteis:"
    echo "  docker logs lisa_dev              # Ver logs"
    echo "  docker exec -it lisa_dev bash     # Entrar no container"
    echo "  docker stop lisa_dev              # Parar a Lisa"
    echo ""
    echo "🔄 Para rebuild:"
    echo "  docker exec lisa_dev bash -c 'cd /workspace && source /opt/ros/humble/setup.bash && colcon build --symlink-install'"
else
    echo "❌ Erro ao iniciar o container!"
    exit 1
fi 