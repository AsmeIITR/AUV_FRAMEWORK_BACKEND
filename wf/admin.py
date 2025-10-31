"""
Admin configuration for wf app
"""

from django.contrib import admin
from .models import ParameterSnapshot, TelemetryLog


@admin.register(ParameterSnapshot)
class ParameterSnapshotAdmin(admin.ModelAdmin):
    list_display = ['id', 'timestamp', 'parameter_count', 'notes_preview']
    list_filter = ['timestamp']
    search_fields = ['notes']
    readonly_fields = ['timestamp', 'parameters']
    date_hierarchy = 'timestamp'

    def parameter_count(self, obj):
        return len(obj.parameters) if obj.parameters else 0
    parameter_count.short_description = 'Parameters'

    def notes_preview(self, obj):
        return obj.notes[:50] + '...' if len(obj.notes) > 50 else obj.notes
    notes_preview.short_description = 'Notes'


@admin.register(TelemetryLog)
class TelemetryLogAdmin(admin.ModelAdmin):
    list_display = [
        'id', 'timestamp', 'altitude', 'latitude', 'longitude',
        'battery_voltage', 'battery_remaining'
    ]
    list_filter = ['timestamp']
    readonly_fields = ['timestamp']
    date_hierarchy = 'timestamp'

    fieldsets = (
        ('Timestamp', {
            'fields': ('timestamp',)
        }),
        ('Position', {
            'fields': ('altitude', 'latitude', 'longitude', 'heading')
        }),
        ('Attitude', {
            'fields': ('roll', 'pitch', 'yaw')
        }),
        ('Battery', {
            'fields': ('battery_voltage', 'battery_current', 'battery_remaining')
        }),
    )
