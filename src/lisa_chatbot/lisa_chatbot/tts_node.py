#!/usr/bin/env python3
"""
TTS Node para ROS2 - Lisa Chatbot
Sintetiza voz usando Piper com modelo Cadu
Assina /lisa_response_text e reproduz o áudio
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import threading
import tempfile
import os
import queue
import time

class TTSNode(Node):
    def __init__(self):
        super().__init__('tts_node')
        
        # Subscriber para respostas da Lisa
        self.response_subscriber = self.create_subscription(
            String,
            '/lisa_response_text',
            self.response_callback,
            10
        )
        
        self.get_logger().info('TTS Node inicializado')
        
        # Configurações do Piper
        self.piper_path = "/app/piper/piper"
        self.model_path = "/app/piper/models/pt_BR-cadu-medium.onnx"
        
        # Verifica se os arquivos do Piper existem
        if not os.path.exists(self.piper_path):
            self.get_logger().error(f'Piper não encontrado em: {self.piper_path}')
        if not os.path.exists(self.model_path):
            self.get_logger().error(f'Modelo não encontrado em: {self.model_path}')
        
        # Buffer para acumular texto
        self.text_buffer = ""
        self.is_processing = False
        self.is_finished = False
        
        # Fila para processamento sequencial de áudio
        self.audio_queue = queue.Queue()
        self.audio_thread = threading.Thread(target=self.audio_worker)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info('TTS Node pronto para sintetizar voz!')
    
    def response_callback(self, msg):
        """Callback para processar respostas recebidas"""
        text = msg.data
        print(f"[DEBUG] TTS recebeu: '{text}'")  # DEBUG
        
        # Controle de fluxo baseado nos marcadores
        # Remove aspas extras que podem vir do ROS topic pub
        clean_text = text.strip().strip("'\"")
        
        if clean_text == "[LISA_PROCESSING]":
            self.get_logger().info('🧠 Lisa está pensando...')
            print("[DEBUG] Modo PROCESSING ativado")  # DEBUG
            self.text_buffer = ""
            self.is_processing = True
            self.is_finished = False
            return
        
        elif clean_text == "[LISA_FINISHED]":
            self.get_logger().info('✅ Lisa terminou de responder')
            print(f"[DEBUG] Modo FINISHED. Buffer: '{self.text_buffer}'")  # DEBUG
            # Processa qualquer texto restante no buffer
            if self.text_buffer.strip():
                print(f"[DEBUG] Processando buffer final: '{self.text_buffer.strip()}'")  # DEBUG
                self.synthesize_text(self.text_buffer.strip())
            self.text_buffer = ""
            self.is_processing = False
            self.is_finished = True
            return
        
        elif clean_text.startswith("[LISA_ERROR]"):
            self.get_logger().error(f'Erro da Lisa: {text}')
            error_text = text.replace("[LISA_ERROR]", "").strip()
            if error_text:
                self.synthesize_text(error_text)
            return
        
        # Texto normal da resposta
        print(f"[DEBUG] Texto normal recebido. is_processing: {self.is_processing}")  # DEBUG
        if self.is_processing:
            print("[DEBUG] Adicionando ao buffer")  # DEBUG
            self.text_buffer += text
            
            # Processa frases completas (terminadas com . ! ? ou com quebra de linha)
            if any(punct in text for punct in ['. ', '! ', '? ', '\n', '.\n', '!\n', '?\n']):
                print("[DEBUG] Detectou fim de frase")  # DEBUG
                sentences = self.split_into_sentences(self.text_buffer)
                
                # Sintetiza todas as frases completas
                for sentence in sentences[:-1]:  # Todas exceto a última (que pode estar incompleta)
                    if sentence.strip():
                        print(f"[DEBUG] Sintetizando frase: '{sentence.strip()}'")  # DEBUG
                        self.synthesize_text(sentence.strip())
                
                # Mantém a última frase no buffer se estiver incompleta
                self.text_buffer = sentences[-1] if sentences else ""
        else:
            print("[DEBUG] Não está em modo processing, ignorando texto")  # DEBUG
    
    def split_into_sentences(self, text):
        """Divide o texto em frases"""
        import re
        # Divide em frases usando pontuação
        sentences = re.split(r'([.!?]+)', text)
        
        # Reconstrói as frases com sua pontuação
        result = []
        for i in range(0, len(sentences)-1, 2):
            sentence = sentences[i].strip()
            if i+1 < len(sentences):
                sentence += sentences[i+1]
            if sentence.strip():
                result.append(sentence.strip())
        
        # Adiciona qualquer texto restante
        if sentences and sentences[-1].strip():
            result.append(sentences[-1].strip())
        
        return result
    
    def synthesize_text(self, text):
        """Adiciona texto à fila para síntese"""
        print(f"[DEBUG] synthesize_text chamado com: '{text}'")  # DEBUG
        if text and len(text.strip()) > 2:  # Só processa textos com conteúdo
            print(f"[DEBUG] Adicionando à fila de áudio: '{text.strip()}'")  # DEBUG
            self.audio_queue.put(text.strip())
        else:
            print(f"[DEBUG] Texto muito curto, ignorando: '{text}'")  # DEBUG
    
    def audio_worker(self):
        """Worker thread para processar áudio sequencialmente"""
        while True:
            try:
                text = self.audio_queue.get(timeout=1.0)
                if text:
                    self.generate_and_play_audio(text)
                self.audio_queue.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Erro no worker de áudio: {e}')
    
    def generate_and_play_audio(self, text):
        """Gera e reproduz áudio usando Piper"""
        try:
            self.get_logger().info(f'🔊 Sintetizando: "{text[:50]}..."')
            
            # Cria arquivo temporário para o áudio
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as temp_file:
                temp_path = temp_file.name
            
            try:
                # Comando do Piper para síntese
                piper_cmd = [
                    self.piper_path,
                    '--model', self.model_path,
                    '--output_file', temp_path
                ]
                
                # Executa o Piper
                process = subprocess.Popen(
                    piper_cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True
                )
                
                stdout, stderr = process.communicate(input=text)
                
                if process.returncode != 0:
                    self.get_logger().error(f'Erro no Piper: {stderr}')
                    return
                
                # Reproduz o áudio usando paplay (PulseAudio) ou aplay (ALSA)
                if os.path.exists(temp_path):
                    # Tenta primeiro com paplay (PulseAudio)
                    try:
                        play_cmd = ['paplay', temp_path]
                        subprocess.run(play_cmd, check=True, 
                                     stdout=subprocess.DEVNULL, 
                                     stderr=subprocess.DEVNULL)
                        self.get_logger().info('🎵 Áudio reproduzido com PulseAudio')
                    except (subprocess.CalledProcessError, FileNotFoundError):
                        # Se paplay falhar, tenta com aplay
                        try:
                            play_cmd = ['aplay', '-D', 'plughw:1,0', temp_path]
                            subprocess.run(play_cmd, check=True, 
                                         stdout=subprocess.DEVNULL, 
                                         stderr=subprocess.DEVNULL)
                            self.get_logger().info('🎵 Áudio reproduzido com ALSA')
                        except subprocess.CalledProcessError:
                            self.get_logger().error('Falha ao reproduzir áudio com PulseAudio e ALSA')
                else:
                    self.get_logger().error('Arquivo de áudio não foi criado')
                    
            finally:
                # Remove arquivo temporário
                if os.path.exists(temp_path):
                    os.unlink(temp_path)
                    
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'Erro ao reproduzir áudio: {e}')
        except Exception as e:
            self.get_logger().error(f'Erro na síntese de voz: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    tts_node = TTSNode()
    
    try:
        rclpy.spin(tts_node)
    except KeyboardInterrupt:
        pass
    finally:
        tts_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
