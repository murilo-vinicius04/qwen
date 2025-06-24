# 🤖 Lisa Chatbot - ROS2 TTS System

Sistema de síntese de voz (Text-to-Speech) para chatbot usando ROS2 e Piper TTS com voz brasileira.

## 📋 Características

- **Voz**: Cadu (Brasileiro - Masculino)
- **Engine TTS**: Piper (rhasspy)
- **Framework**: ROS2 Humble
- **Áudio**: ALSA com suporte completo
- **Containerização**: Docker com permissões de áudio
- **Tempo Real**: Síntese e reprodução instantânea

## 🏗️ Arquitetura

```
┌─────────────────┐    /user_question_text    ┌─────────────────┐
│   STT Node      │ ──────────────────────────▶│   LLM Node      │
│ (Speech-to-Text)│                            │ (Language Model)│
└─────────────────┘                            └─────────────────┘
                                                         │
                                                         ▼
                                               /lisa_response_text
                                                         │
                                                         ▼
                                               ┌─────────────────┐
                                               │   TTS Node      │
                                               │(Text-to-Speech) │
                                               └─────────────────┘
                                                         │
                                                         ▼
                                                   🔊 Áudio
```

## 🚀 Início Rápido

### 🐳 Opção 1: Docker (Recomendado)

#### 1. Build da Imagem Docker
```bash
docker build -t lisa_chatbot .
```

#### 2. Executar Lisa (Background)
```bash
./run_lisa.sh
```

#### 3. Testar TTS
```bash
./test_tts.sh
```

### 🛠️ Opção 2: Instalação Local

#### 1. Instalar Dependências
```bash
# Instalar ROS2 Humble
sudo apt update && sudo apt install -y ros-humble-desktop

# Instalar dependências Python
pip3 install -r requirements.txt

# Instalar Piper TTS (manual)
# Baixe de: https://github.com/rhasspy/piper/releases
```

#### 2. Build do Workspace
```bash
# No diretório lisa_ws/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

#### 3. Executar
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lisa_chatbot lisa.launch.py
```

## 📜 Scripts Disponíveis

### `run_lisa.sh`
Executa a Lisa em background com todas as permissões de áudio.

### `run_interactive.sh`
Executa a Lisa em modo interativo para desenvolvimento e debug.

### `test_tts.sh`
Testa o sistema TTS enviando uma mensagem de exemplo.

## 🔧 Comandos Úteis

### Verificar Status
```bash
docker ps                    # Ver containers rodando
docker logs lisa_running     # Ver logs da Lisa
```

### Entrar no Container
```bash
docker exec -it lisa_running bash
```

### Parar Lisa
```bash
docker stop lisa_running
docker rm lisa_running
```

## 🎯 Uso do Sistema TTS

O sistema TTS funciona com um protocolo de 3 etapas:

### 1. Sinal de Início
```bash
ros2 topic pub --once /lisa_response_text std_msgs/String '{data: "[LISA_PROCESSING]"}'
```

### 2. Envio do Texto
```bash
ros2 topic pub --once /lisa_response_text std_msgs/String '{data: "Sua mensagem aqui"}'
```

### 3. Sinal de Fim
```bash
ros2 topic pub --once /lisa_response_text std_msgs/String '{data: "[LISA_FINISHED]"}'
```

## 🎵 Configuração de Áudio

O sistema está configurado para:
- **Dispositivo**: `plughw:1,0` (ALSA)
- **Formato**: 16-bit, 22050 Hz, Mono
- **Permissões**: Usuário `1000:29` com grupo `audio`

## 🔍 Troubleshooting

### Sem áudio?
1. Verifique o volume do sistema
2. Confirme que o usuário está no grupo `audio`: `groups $USER`
3. Teste áudio direto: `aplay /usr/share/sounds/alsa/Front_Left.wav`

### Container não inicia?
1. Verifique se a imagem foi construída: `docker images | grep lisa_chatbot`
2. Verifique permissões do Docker: `docker run hello-world`

### TTS não funciona?
1. Verifique logs: `docker logs lisa_running`
2. Teste manualmente: `./test_tts.sh`
3. Entre no container: `./run_interactive.sh`

## 📁 Estrutura do Projeto

```
lisa_ws/                    # Workspace ROS2
├── src/                    # Código fonte (padrão ROS2)
│   └── lisa_chatbot/       # Pacote ROS2
│       ├── lisa_chatbot/   # Módulo Python
│       │   ├── __init__.py
│       │   ├── llm_node.py        # Nó do modelo de linguagem
│       │   ├── stt_node.py        # Nó de reconhecimento de voz
│       │   └── tts_node.py        # Nó de síntese de voz
│       ├── launch/
│       │   └── lisa.launch.py     # Arquivo de launch ROS2
│       ├── package.xml           # Manifesto ROS2
│       └── setup.py              # Setup do pacote
├── build/                  # Arquivos de build (gerado automaticamente)
├── install/                # Arquivos instalados (gerado automaticamente)
├── log/                    # Logs de build (gerado automaticamente)
├── Dockerfile             # Imagem Docker principal
├── requirements.txt       # Dependências Python
├── run_lisa.sh           # Script principal
├── run_interactive.sh    # Script interativo
├── test_tts.sh          # Script de teste
└── README.md            # Este arquivo
```

## 🎤 Modelo de Voz

- **Nome**: Cadu
- **Idioma**: Português Brasileiro
- **Gênero**: Masculino
- **Qualidade**: Média (balance entre qualidade e performance)
- **Arquivo**: `pt_BR-cadu-medium.onnx`

## 🔒 Requisitos de Sistema

- **Docker**: Versão 20.10+
- **Sistema**: Linux com ALSA
- **Memória**: Mínimo 2GB RAM
- **Espaço**: ~3GB para imagem Docker
- **Áudio**: Dispositivo de áudio funcional

## 🤝 Contribuição

Para contribuir com o projeto:

1. Fork o repositório
2. Crie uma branch: `git checkout -b feature/nova-funcionalidade`
3. Commit suas mudanças: `git commit -m 'Adiciona nova funcionalidade'`
4. Push para a branch: `git push origin feature/nova-funcionalidade`
5. Abra um Pull Request

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo `LICENSE` para mais detalhes.

---

**🎉 Lisa Chatbot está pronta para conversar com você!**
