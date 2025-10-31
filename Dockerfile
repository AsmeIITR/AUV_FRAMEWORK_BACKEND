FROM --platform=$BUILDPLATFORM python:3.11-slim

WORKDIR /app

COPY . .
COPY .env /app/.env

RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 8000
CMD ["python", "manage.py", "runserver", "0.0.0.0:3000"]
