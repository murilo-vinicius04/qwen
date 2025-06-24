#!/usr/bin/env python3
"""
STT Node para ROS2 - Lisa Chatbot
Converte voz em texto usando faster-whisper com webrtcvad
Publica texto reconhecido em /user_question_text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pyaudio
import wave
import webrtcvad
import numpy as np
from faster_whisper import WhisperModel
import threading
import collections
import time
import os

class STTNode(Node):
    def __init__(self):
        super().__init__('stt_node')
        
        # Publisher para o texto reconhecido
        self.text_publisher = self.create_publisher(String, '/user_question_text', 10)
        
        self.get_logger().info('STT Node inicializado')
        
        # Configurações de áudio
        self.CHUNK = 320  # 20ms de áudio a 16kHz
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RATE = 16000
        self.RECORD_SECONDS = 0.02  # 20ms frames
        
        # Configurações do VAD
        self.vad = webrtcvad.Vad(2)  # Sensibilidade 0-3 (2 = moderado)
        self.frame_duration_ms = 20
        self.padding_duration_ms = 300
        self.num_padding_frames = int(self.padding_duration_ms / self.frame_duration_ms)
        
        # Buffer para armazenar frames de áudio
        self.ring_buffer = collections.deque(maxlen=self.num_padding_frames)
        self.triggered = False
        self.voiced_frames = []
        
        # Modelo Whisper
        self.get_logger().info('Carregando modelo Whisper...')
        try:
            self.whisper_model = WhisperModel("small", device="cpu", compute_type="int8")
            self.get_logger().info('Modelo Whisper carregado com sucesso!')
        except Exception as e:
            self.get_logger().error(f'Erro ao carregar Whisper: {e}')
            return
        
        # Inicializa PyAudio
        self.audio = pyaudio.PyAudio()
        
        # Inicia a captura de áudio em thread separada
        self.recording = True
        self.audio_thread = threading.Thread(target=self.audio_capture_loop)
        self.audio_thread.daemon = True
        self.audio_thread.start()
        
        self.get_logger().info('STT Node pronto! Fale para começar...')
    
    def audio_capture_loop(self):
        """Loop principal de captura de áudio"""
        try:
            stream = self.audio.open(
                format=self.FORMAT,
                channels=self.CHANNELS,
                rate=self.RATE,
                input=True,
                frames_per_buffer=self.CHUNK
            )
            
            self.get_logger().info('Gravação iniciada. Aguardando fala...')
            
            while self.recording:
                try:
                    # Lê frame de áudio
                    frame = stream.read(self.CHUNK, exception_on_overflow=False)
                    
                    # Verifica se há voz usando VAD
                    is_speech = self.vad.is_speech(frame, self.RATE)
                    
                    if not self.triggered:
                        # Não está gravando ainda
                        self.ring_buffer.append((frame, is_speech))
                        num_voiced = len([f for f, speech in self.ring_buffer if speech])
                        
                        # Inicia gravação se detectar voz suficiente
                        if num_voiced > 0.9 * self.ring_buffer.maxlen:
                            self.triggered = True
                            self.get_logger().info('🎤 Detectei voz! Gravando...')
                            self.voiced_frames.extend([f for f, s in self.ring_buffer])
                            self.ring_buffer.clear()
                    else:
                        # Está gravando
                        self.voiced_frames.append(frame)
                        self.ring_buffer.append((frame, is_speech))
                        num_unvoiced = len([f for f, speech in self.ring_buffer if not speech])
                        
                        # Para gravação se detectar silêncio suficiente
                        if num_unvoiced > 0.9 * self.ring_buffer.maxlen:
                            self.triggered = False
                            self.get_logger().info('🔇 Silêncio detectado. Processando áudio...')
                            self.process_audio()
                            self.voiced_frames = []
                            self.ring_buffer.clear()
                            
                except Exception as e:
                    self.get_logger().warn(f'Erro na captura de áudio: {e}')
                    time.sleep(0.1)
            
            stream.stop_stream()
            stream.close()
            
        except Exception as e:
            self.get_logger().error(f'Erro no stream de áudio: {e}')
    
    def process_audio(self):
        """Processa o áudio capturado e converte para texto"""
        if not self.voiced_frames:
            return
        
        try:
            # Converte frames para numpy array
            audio_data = b''.join(self.voiced_frames)
            audio_np = np.frombuffer(audio_data, dtype=np.int16).astype(np.float32) / 32768.0
            
            # Transcribe com Whisper
            self.get_logger().info('🧠 Convertendo fala para texto...')
            segments, info = self.whisper_model.transcribe(
                audio_np, 
                language="pt",
                beam_size=5,
                best_of=5,
                temperature=0.0
            )
            
            # Extrai o texto
            transcribed_text = ""
            for segment in segments:
                transcribed_text += segment.text
            
            transcribed_text = transcribed_text.strip()
            
            if transcribed_text and len(transcribed_text) > 2:
                self.get_logger().info(f'📝 Texto reconhecido: "{transcribed_text}"')
                
                # Publica o texto reconhecido
                msg = String()
                msg.data = transcribed_text
                self.text_publisher.publish(msg)
            else:
                self.get_logger().info('🤐 Nenhum texto reconhecido')
                
        except Exception as e:
            self.get_logger().error(f'Erro ao processar áudio: {e}')
    
    def destroy_node(self):
        """Limpa recursos ao destruir o nó"""
        self.recording = False
        if hasattr(self, 'audio_thread'):
            self.audio_thread.join(timeout=1.0)
        if hasattr(self, 'audio'):
            self.audio.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    stt_node = STTNode()
    
    try:
        rclpy.spin(stt_node)
    except KeyboardInterrupt:
        pass
    finally:
        stt_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
