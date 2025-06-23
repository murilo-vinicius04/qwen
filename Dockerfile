# 1. IMAGEM BASE
# Começamos com uma imagem oficial do Python, versão 3.11. É limpa e confiável.
FROM python:3.11-slim

# 2. VARIÁVEIS DE AMBIENTE
# Evita que o instalador de pacotes fique fazendo perguntas.
ENV DEBIAN_FRONTEND=noninteractive
# Força a compilação do llama-cpp-python com OpenMP.
ENV CMAKE_ARGS="-DLLAMA_OPENMP=ON"
# Desativa o cache do pip dentro da imagem pra garantir que tudo seja fresco.
ENV PIP_NO_CACHE_DIR=off

# 3. INSTALANDO DEPENDÊNCIAS DO SISTEMA
# Aqui a gente instala o compilador, a FAMOSA libgomp1, e o git.
# É a primeira coisa que a gente faz no ambiente limpo.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    pkg-config \
    libgomp1 \
    git \
    && rm -rf /var/lib/apt/lists/*

# 4. PREPARANDO O APP
# Cria uma pasta para o nosso código dentro da "caixa".
WORKDIR /app

# 5. INSTALANDO DEPENDÊNCIAS DO PYTHON
# Copia SÓ a lista de requisitos primeiro. O Docker é esperto, se esse arquivo não mudar,
# ele não vai re-instalar tudo toda vez, acelerando o processo.
COPY requirements.txt .

# Agora sim, instalamos os pacotes Python.
# O llama-cpp-python vai compilar aqui, no nosso ambiente perfeito.
RUN pip install --no-cache-dir -r requirements.txt

# 6. COPIANDO O CÓDIGO DO APP
# Com tudo já instalado, copiamos o resto do nosso código (o test.py).
COPY . .
