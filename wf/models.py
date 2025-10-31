"""
Models for storing Pixhawk telemetry data
"""

from django.db import models


class ParameterSnapshot(models.Model):
    timestamp = models.DateTimeField(auto_now_add=True)
    parameters = models.JSONField()
    notes = models.TextField(blank=True)

    class Meta:
        ordering = ['-timestamp']
        verbose_name = 'Parameter Snapshot'
        verbose_name_plural = 'Parameter Snapshots'

    def __str__(self):
        return f"Snapshot at {self.timestamp}"


class TelemetryLog(models.Model):
    timestamp = models.DateTimeField(auto_now_add=True)
    altitude = models.FloatField(null=True, blank=True)
    latitude = models.FloatField(null=True, blank=True)
    longitude = models.FloatField(null=True, blank=True)
    heading = models.FloatField(null=True, blank=True)
    roll = models.FloatField(null=True, blank=True)
    pitch = models.FloatField(null=True, blank=True)
    yaw = models.FloatField(null=True, blank=True)
    battery_voltage = models.FloatField(null=True, blank=True)
    battery_current = models.FloatField(null=True, blank=True)
    battery_remaining = models.IntegerField(null=True, blank=True)

    class Meta:
        ordering = ['-timestamp']
        verbose_name = 'Telemetry Log'
        verbose_name_plural = 'Telemetry Logs'
        indexes = [
            models.Index(fields=['-timestamp']),
        ]

    def __str__(self):
        return f"Log at {self.timestamp}"
