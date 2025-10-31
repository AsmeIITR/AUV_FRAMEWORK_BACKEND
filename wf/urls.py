from django.urls import path, include
from rest_framework.routers import DefaultRouter
# from . import views
#
router = DefaultRouter()
# router.register(r'snapshots', views.ParameterSnapshotViewSet,
#                 basename='snapshot')
# router.register(r'logs', views.TelemetryLogViewSet, basename='log')
# router.register(r'parameters', views.ParameterViewSet, basename='parameter')

urlpatterns = [
    path('', include(router.urls)),
]
