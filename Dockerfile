# 1. IMAGEM BASE
# Começamos com ROS2 Humble que já tem Python 3.10
FROM ros:humble-ros-core-jammy

# 2. VARIÁVEIS DE AMBIENTE
# Evita que o instalador de pacotes fique fazendo perguntas.
ENV DEBIAN_FRONTEND=noninteractive
# Configura o ROS2
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# 3. INSTALANDO DEPENDÊNCIAS DO SISTEMA
# Instala dependências para Python, ROS2, audio, e Piper
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    git \
    python3-pip \
    python3-dev \
    python3-venv \
    ros-humble-rclpy \
    ros-humble-std-msgs \
    alsa-utils \
    portaudio19-dev \
    python3-pyaudio \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    espeak-ng \
    espeak-ng-data \
    pulseaudio \
    pulseaudio-utils \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# 4. PREPARANDO O WORKSPACE ROS2
WORKDIR /app

# Instala python3-colcon-common-extensions para ROS2
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 5. INSTALANDO DEPENDÊNCIAS DO PYTHON
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# 6. BAIXA PIPER TTS
RUN mkdir -p /app/piper && \
    wget -O /app/piper/piper_amd64.tar.gz https://github.com/rhasspy/piper/releases/download/v1.2.0/piper_amd64.tar.gz && \
    tar -xzf /app/piper/piper_amd64.tar.gz -C /app/piper --strip-components=1 && \
    rm /app/piper/piper_amd64.tar.gz

# Baixa modelo de voz em português brasileiro
RUN mkdir -p /app/piper/models && \
    wget -O /app/piper/models/pt_BR-cadu-medium.onnx https://huggingface.co/rhasspy/piper-voices/resolve/main/pt/pt_BR/cadu/medium/pt_BR-cadu-medium.onnx && \
    wget -O /app/piper/models/pt_BR-cadu-medium.onnx.json https://huggingface.co/rhasspy/piper-voices/resolve/main/pt/pt_BR/cadu/medium/pt_BR-cadu-medium.onnx.json

# 7. CRIA WORKSPACE ROS2 E COPIA CÓDIGO
# Copia a estrutura completa do workspace
COPY src/ /app/ros2_ws/src/

# 8. COMPILA O WORKSPACE ROS2
WORKDIR /app/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# 9. COMANDO PADRÃO - Executa launch file
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch lisa_chatbot lisa.launch.py"]
