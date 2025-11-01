# AUV Framework Setup

## Prerequisites
- Docker
- Docker Compose
---

### Run

```bash

curl -L -o docker-compose.yml https://raw.githubusercontent.com/AsmeIITR/AUV_FRAMEWORK_BACKEND/master/docker-compose.yml
touch .env

# Insert the config in env
# DJANGO_SECRET_KEY= <KEY>
# DJANGO_DEBUG=True
# DJANGO_ALLOWED_HOSTS=*
# MAVLINK_CONNECTION=/dev/ttyACM0

docker compose up --build

```
