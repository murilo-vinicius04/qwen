import langchain
from langchain_core.prompts import ChatPromptTemplate, MessagesPlaceholder
from langchain.memory import ConversationBufferWindowMemory
from langchain_openai import ChatOpenAI
from langchain_core.runnables.history import RunnableWithMessageHistory
import argparse # <<<<<<<<<<<<<<<<< Nova importação

# =================================================================
# Configuração do Argumento de Linha de Comando (NOVO!)
# =================================================================
parser = argparse.ArgumentParser(description="Chatbot Qwen com opções de debug.")
parser.add_argument(
    "--no-debug", 
    action="store_true", # Se a flag for usada, armazena True
    help="Desativa as mensagens de debug detalhadas da LangChain."
)
args = parser.parse_args()

# Ligue ou desligue o modo debug com base no argumento
if args.no_debug:
    langchain.debug = False
    print("Modo debug da LangChain DESATIVADO. Apenas perguntas e respostas serão exibidas.\n")
else:
    langchain.debug = True
    print("Modo debug da LangChain ATIVADO. Mensagens detalhadas de execução serão exibidas.\n")


# =================================================================
# 1. CONFIGURAÇÃO DA CONEXÃO (Não muda)
# =================================================================
llm = ChatOpenAI(
    base_url="http://100.103.176.123:11434/v1",
    api_key="not-needed",
    temperature=0.7,
)

# =================================================================
# 2. O NOVO PROMPT (Estilo Chat)
# =================================================================
# Em vez de um template de string única, usamos um ChatPromptTemplate.
# Ele entende a diferença entre mensagens do sistema, do histórico e do usuário.
prompt = ChatPromptTemplate.from_messages(
    [
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
        MessagesPlaceholder(variable_name="history"), # Onde o histórico será injetado
        ("human", "{input}"), # A nova pergunta do usuário
    ]
)

# =================================================================
# 3. A NOVA CHAIN COM MEMÓRIA INTEGRADA
# =================================================================
# Chain principal, sem se preocupar com memória ainda.
runnable = prompt | llm

# Dicionário para guardar os históricos de cada conversa.
# A chave ("abc") é o ID da sessão. Poderíamos ter várias conversas ao mesmo tempo.
store = {}

def get_session_history(session_id: str):
    """Função que busca o histórico da sessão ou cria um novo."""
    if session_id not in store:
        # A memória que a gente cria, e a RunnableWithMessageHistory
        # vai interagir com o 'chat_memory' dela.
        store[session_id] = ConversationBufferWindowMemory(k=4, return_messages=True)
    
    # A sacada é retornar o 'chat_memory' que é o objeto ChatMessageHistory de fato
    # que a RunnableWithMessageHistory espera.
    return store[session_id].chat_memory

# A MÁGICA FINAL: Envolvemos nossa chain com o gerenciador de memória.
# Ele vai chamar a função get_session_history para carregar/salvar o histórico.
chain_with_history = RunnableWithMessageHistory(
    runnable,
    get_session_history,
    input_messages_key="input",
    history_messages_key="history",
)

# =================================================================
# 4. LOOP PRINCIPAL (Com Streaming!)
# =================================================================
if __name__ == "__main__":
    # Mensagem inicial atualizada para refletir o nome "Lisa"
    print("🤖 Lisa com memória e STREAMING está online! Mande 'sair' pra finalizar.")
    while True:
        pergunta = input("❓ Sua pergunta: ")
        if pergunta.lower() in ["sair", "exit", "q"]:
            break

        print("\n🧠 Lisa respondeu:") # Atualizado para Lisa
        resposta_completa = "" # Variável para acumular a resposta completa (se precisar)

        # Usa .stream() em vez de .invoke()
        # Cada 'chunk' é um pedacinho da resposta que vai chegando
        for chunk in chain_with_history.stream(
            {"input": pergunta},
            config={"configurable": {"session_id": "conversa_unica"}}
        ):
            # O chunk já vem como o conteúdo da mensagem
            # 'end=""' faz ele não pular linha e 'flush=True' garante que imprime na hora
            print(chunk.content, end="", flush=True)
            resposta_completa += chunk.content # Acumula para ter a resposta completa no final

        print("\n") # Pula uma linha no final para organizar o terminal
        # Opcional: Se precisar da resposta completa em uma variável:
        # print(f"DEBUG: Resposta completa acumulada: {resposta_completa}")