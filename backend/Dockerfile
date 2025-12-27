# Dockerfile for Physical AI Textbook RAG Backend
# Optimized for Railway deployment

FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements first for layer caching
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Copy docs directory for content ingestion (seeding Qdrant)
COPY ../docs /app/docs

# Expose port (Railway will assign PORT dynamically)
EXPOSE ${PORT:-8001}

# Health check (use Railway's assigned PORT)
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:${PORT:-8001}/api/health || exit 1

# Run the application
# Railway will set PORT env var, fallback to 8001 for local development
CMD uvicorn app.main:app --host 0.0.0.0 --port ${PORT:-8001}
