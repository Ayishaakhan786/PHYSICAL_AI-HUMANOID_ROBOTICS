FROM python:3.11-slim

WORKDIR /app

# System dependencies for Qdrant/Python
RUN apt-get update && apt-get install -y build-essential && rm -rf /var/lib/apt/lists/*

# Backend folder se requirements copy karein
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Backend ka sara code /app mein copy karein
COPY backend/ .

# Port 7860 HF ka default hai
EXPOSE 7860

CMD ["uvicorn", "api:app", "--host", "0.0.0.0", "--port", "7860"]