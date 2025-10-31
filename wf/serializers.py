from rest_framework import serializers
from .models import ParameterSnapshot, TelemetryLog


class ParameterSnapshotSerializer(serializers.ModelSerializer):
    parameter_count = serializers.SerializerMethodField()

    class Meta:
        model = ParameterSnapshot
        fields = ['id', 'timestamp', 'parameters', 'notes', 'parameter_count']
        read_only_fields = ['timestamp']

    def get_parameter_count(self, obj):
        return len(obj.parameters) if obj.parameters else 0


class TelemetryLogSerializer(serializers.ModelSerializer):
    class Meta:
        model = TelemetryLog
        fields = [
            'id', 'timestamp', 'altitude', 'latitude', 'longitude',
            'heading', 'roll', 'pitch', 'yaw', 'battery_voltage',
            'battery_current', 'battery_remaining'
        ]
        read_only_fields = ['timestamp']


class ParameterSerializer(serializers.Serializer):
    name = serializers.CharField(max_length=16)
    value = serializers.FloatField()
