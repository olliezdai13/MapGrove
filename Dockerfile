# Use official Python runtime as base
FROM python:3.11-slim

# Set working directory in container
WORKDIR /app

# Copy requirements first (for caching)
COPY requirements.txt .

# Install Python dependencies
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of your code
COPY . .

# Command to run your app (adjust as needed)
CMD ["python", "PythonCode/main.py"]
