# Dockerfile for Hugging Face Spaces deployment
FROM python:3.11-slim

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    gcc \
    g++ \
    make \
    libpq-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy requirements file
COPY requirements-hf.txt requirements.txt

# Install Python dependencies
# Use --no-cache-dir to reduce image size
RUN pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY . .

# Create data directory for SQLite sessions
RUN mkdir -p data

# Set environment variables
ENV PYTHONUNBUFFERED=1
ENV PORT=7860

# Expose Hugging Face Spaces default port
EXPOSE 7860

# Health check
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD curl -f http://localhost:7860/health || exit 1

# Run the application
# Bind to 0.0.0.0:7860 (HF Spaces requirement)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "7860"]
