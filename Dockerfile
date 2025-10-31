FROM --platform=$BUILDPLATFORM python:3.11-slim

WORKDIR /app

COPY . .

RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --no-cache-dir -r requirements.txt

EXPOSE 3000
CMD ["python", "manage.py", "migrate"]
CMD ["daphne", "-b", "0.0.0.0", "-p", "3000", "auv25.asgi:application"]
