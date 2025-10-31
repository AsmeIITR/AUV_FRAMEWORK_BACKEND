from django.urls import re_path
from . import consumers

websocket_urlpatterns = [
    re_path(r'ws/pixhawk/$', consumers.PixhawkDataConsumer.as_asgi()),
    re_path(r'ws/simulated/$', consumers.SimulatedPixhawkDataConsumer.as_asgi()),
]
