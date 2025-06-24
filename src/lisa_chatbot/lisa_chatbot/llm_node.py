#!/usr/bin/env python3
"""
LLM Node para ROS2 - Lisa Chatbot
Assina /user_question_text e publica respostas em partes no /lisa_response_text
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import langchain
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain.memory import ConversationBufferWindowMemory
from langchain_openai import ChatOpenAI
from langchain_core.runnables.history import RunnableWithMessageHistory
import threading
import time

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        
        # Publisher para as respostas
        self.response_publisher = self.create_publisher(String, '/lisa_response_text', 10)
        
        # Subscriber para as perguntas
        self.question_subscriber = self.create_subscription(
            String,
            '/user_question_text',
            self.question_callback,
            10
        )
        
        self.get_logger().info('LLM Node inicializado. Aguardando perguntas em /user_question_text')
        
        # Configuração da LLM (mesmo código do original)
        self.setup_llm()
        
    def setup_llm(self):
        """Configura a LLM com memória e prompt"""
        # Desativa debug por padrão
        langchain.debug = False
        
        # Configuração da conexão com Qwen
        self.llm = ChatOpenAI(
            base_url="http://100.103.176.123:11434/v1",
            api_key="not-needed",
            temperature=0.7,
        )
        
        # Prompt do sistema (mesmo do original)
        self.prompt = ChatPromptTemplate.from_messages([
            (
                "system",
                """
                Você é a Lisa, um robô educacional. Sua missão é ser o equilíbrio perfeito entre um cientista preciso e um professor super gente boa e didático.

                Siga estas regras:
                1.  **Professor Divertido:** Use analogias e metáforas para explicar temas complexos.
                2.  **Conversa Natural:** Se o usuário usar gírias ou falar de modo informal, entre na onda! Responda de forma amigável e natural.
                3.  **Confie no Contexto:** Assuma o contexto da conversa. Evite pedir confirmações óbvias.
                4.  **Correção é Prioridade:** Explique a ciência de forma simples, mas sempre correta.

                ---
                ### Exemplo de Diálogo Ideal:

                Humano: E aí, Lisa, firmeza? Me explica rapidão como funciona um motor de carro.

                Lisa: E aí! Firmeza total. Bora lá! Pensa no motor como uma pipoqueira super potente. Você coloca o 'milho' (que é uma mistura de ar e gasolina), dá uma 'faísca' (a vela de ignição), e PÁ! A explosão empurra uma pecinha (o pistão) pra baixo com uma força bruta. Fazendo isso milhares de vezes por minuto, em vários 'potinhos' diferentes, a gente faz as rodas girarem. Sacou a analogia?
                ---

                Agora, continue a conversa abaixo.

                Histórico da Conversa:
                {history}

                Humano: {input}
                Lisa:
                """,
            ),
            MessagesPlaceholder(variable_name="history"),
            ("human", "{input}"),
        ])
        
        # Chain principal
        runnable = self.prompt | self.llm
        
        # Dicionário para guardar históricos
        self.store = {}
        
        # Chain com memória
        self.chain_with_history = RunnableWithMessageHistory(
            runnable,
            self.get_session_history,
            input_messages_key="input",
            history_messages_key="history",
        )
        
    def get_session_history(self, session_id: str):
        """Função que busca o histórico da sessão ou cria um novo."""
        if session_id not in self.store:
            self.store[session_id] = ConversationBufferWindowMemory(k=4, return_messages=True)
        return self.store[session_id].chat_memory
    
    def question_callback(self, msg):
        """Callback para processar perguntas recebidas"""
        question = msg.data
        self.get_logger().info(f'Pergunta recebida: {question}')
        
        # Processa a pergunta em uma thread separada para não bloquear
        thread = threading.Thread(target=self.process_question, args=(question,))
        thread.daemon = True
        thread.start()
    
    def process_question(self, question):
        """Processa a pergunta e publica a resposta em streaming"""
        try:
            self.get_logger().info('Processando pergunta com a LLM...')
            
            # Publica indicador de que começou a processar
            start_msg = String()
            start_msg.data = "[LISA_PROCESSING]"
            self.response_publisher.publish(start_msg)
            
            # Stream da resposta
            for chunk in self.chain_with_history.stream(
                {"input": question},
                config={"configurable": {"session_id": "lisa_session"}}
            ):
                # Publica cada pedaço da resposta
                response_msg = String()
                response_msg.data = chunk.content
                self.response_publisher.publish(response_msg)
                
                # Pequeno delay para não sobrecarregar
                time.sleep(0.01)
            
            # Publica indicador de fim
            end_msg = String()
            end_msg.data = "[LISA_FINISHED]"
            self.response_publisher.publish(end_msg)
            
            self.get_logger().info('Resposta enviada com sucesso!')
            
        except Exception as e:
            self.get_logger().error(f'Erro ao processar pergunta: {str(e)}')
            error_msg = String()
            error_msg.data = f"[LISA_ERROR] Desculpe, tive um problema: {str(e)}"
            self.response_publisher.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    
    llm_node = LLMNode()
    
    try:
        rclpy.spin(llm_node)
    except KeyboardInterrupt:
        pass
    finally:
        llm_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
