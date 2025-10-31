from channels.generic.websocket import AsyncWebsocketConsumer
import json
import asyncio
from .mavlink_service import MAVLinkService
from .extended_mavlink_service import ExtendedMAVLinkService
from pymavlink import mavutil
from django.conf import settings


class PixhawkDataConsumer(AsyncWebsocketConsumer):
    async def connect(self):
        await self.accept()
        self.running = True
        """
            Check the connection method in .env
                - Serial: '/dev/ttyUSB0' or 'COM3'
                - UDP: 'udpin:127.0.0.1:14550'
                - TCP: 'tcp:127.0.0.1:5760'
        """
        self.connection_string = settings.MAVLINK_CONNECTION
        self.master = mavutil.mavlink_connection(self.connection_string)
        self.telemetry_service = ExtendedMAVLinkService(self.master)
        # self.param_service = MAVLinkService(self.master)

        self.telemetry_task = asyncio.create_task(self.telemetry_stream())
        # self.parameter_task = asyncio.create_task(self.parameter_stream())

        print(f"[CONNECTION] Pixhawk connected: {self.channel_name}")

    async def disconnect(self, close_code):
        self.running = False

        if hasattr(self, 'telemetry_task'):
            self.telemetry_task.cancel()

        if hasattr(self, 'parameter_task'):
            self.parameter_task.cancel()

        if hasattr(self, 'telemetry_service'):
            self.telemetry_service.close()

        if hasattr(self, 'param_service'):
            self.param_service.close()

        print(f"[CONNECTION] Pixhawk disconnected: {self.channel_name}")

    async def telemetry_stream(self):
        while self.running:
            try:
                telemetry = await asyncio.to_thread(self._get_telemetry)
                await self.send(json.dumps({
                    'type': 'telemetry',
                    'data': telemetry
                }))

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[STREAM] Error in telemetry_stream: {e}")
                await asyncio.sleep(1)

    async def parameter_stream(self):

        while self.running:
            try:
                parameters = await asyncio.to_thread(
                    self.param_service.get_all_parameters
                )

                await self.send(json.dumps({
                    'type': 'parameters',
                    'data': parameters
                }))

            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"[STREAM] Error in parameter_stream: {e}")
                await asyncio.sleep(5)

    def _get_telemetry(self):
        return {
            'attitude': self.telemetry_service.get_attitude(),
            # 'gps': self.telemetry_service.get_gps_raw(),
            # 'global_position': self.telemetry_service.get_global_position(),
            'battery': self.telemetry_service.get_battery_status(),
            'vfr_hud': self.telemetry_service.get_vfr_hud(),
            # 'sys_status': self.telemetry_service.get_sys_status(),
            # 'heartbeat': self.telemetry_service.get_heartbeat(),
            'altitude': self.telemetry_service.get_altitude(),
            'rc_channels': self.telemetry_service.get_rc_channels(),
        }

    async def receive(self, text_data):
        try:
            data = json.loads(text_data)
            command = data.get('command')

            if command == 'set_parameter':
                param_name = data.get('name')
                param_value = data.get('value')
                success = await asyncio.to_thread(
                    self.param_service.set_parameter,
                    param_name,
                    param_value
                )
                await self.send(json.dumps({
                    'type': 'parameter_set_response',
                    'success': success,
                    'name': param_name
                }))

        except Exception as e:
            await self.send(json.dumps({
                'type': 'error',
                'message': str(e)
            }))


class SimulatedPixhawkDataConsumer(AsyncWebsocketConsumer):

    async def connect(self):
        await self.accept()
        self.running = True
        self.telemetry_task = asyncio.create_task(self.telemetry_stream())
        self.parameter_task = asyncio.create_task(self.parameter_stream())
        print(f"[Simulation] Simulated Dashboard connected: {self.channel_name}")

    async def disconnect(self, close_code):
        self.running = False
        if hasattr(self, 'telemetry_task'):
            self.telemetry_task.cancel()
        if hasattr(self, 'parameter_task'):
            self.parameter_task.cancel()
        print(f"[SIMULATION] Simulated Dashboard disconnected: {
              self.channel_name}")

    async def telemetry_stream(self):
        counter = 0

        while self.running:
            try:
                import math
                t = counter * 0.1

                telemetry = {
                    'attitude': {
                        'roll': math.sin(t * 0.5) * 0.3,
                        'pitch': math.cos(t * 0.3) * 0.2,
                        'yaw': (t * 0.1) % (2 * math.pi),
                        'rollspeed': math.cos(t) * 0.1,
                        'pitchspeed': math.sin(t) * 0.1,
                        'yawspeed': 0.05,
                        'timestamp': counter
                    },
                    'gps': {
                        'fix_type': 3,
                        'latitude': 37.7749 + math.sin(t * 0.01) * 0.0001,
                        'longitude': -122.4194 + math.cos(t * 0.01) * 0.0001,
                        'altitude': 100 + math.sin(t * 0.2) * 10,
                        'eph': 1.2,
                        'epv': 2.1,
                        'velocity': 5 + math.sin(t * 0.3) * 2,
                        'cog': (t * 5) % 360,
                        'satellites_visible': 12
                    },
                    'global_position': {
                        'latitude': 37.7749,
                        'longitude': -122.4194,
                        'altitude_msl': 100 + math.sin(t * 0.2) * 10,
                        'altitude_rel': 50 + math.sin(t * 0.2) * 5,
                        'vx': math.cos(t * 0.3) * 5,
                        'vy': math.sin(t * 0.3) * 5,
                        'vz': math.sin(t * 0.1) * 2,
                        'heading': (t * 5) % 360
                    },
                    'battery': {
                        'voltage': 16.8 - (counter % 100) * 0.001,
                        'current_battery': 15 + math.sin(t * 0.5) * 5,
                        'battery_remaining': max(20, 100 - (counter % 1000) * 0.08),
                        'temperature': 25 + math.sin(t * 0.1) * 5
                    },
                    'vfr_hud': {
                        'airspeed': 15 + math.sin(t * 0.3) * 3,
                        'groundspeed': 14 + math.sin(t * 0.3) * 3,
                        'heading': (t * 5) % 360,
                        'throttle': 65 + math.sin(t * 0.2) * 10,
                        'alt': 100 + math.sin(t * 0.2) * 10,
                        'climb': math.sin(t * 0.4) * 3
                    },
                    'sys_status': {
                        'voltage_battery': 16.8,
                        'current_battery': 15.3,
                        'battery_remaining': max(20, 100 - (counter % 1000) * 0.08)
                    },
                    'heartbeat': {
                        'type': 2,
                        'autopilot': 3,
                        'base_mode': 81,
                        'custom_mode': 0,
                        'system_status': 4
                    },
                    'altitude': {
                        'altitude_monotonic': 100 + math.sin(t * 0.2) * 10,
                        'altitude_amsl': 100 + math.sin(t * 0.2) * 10,
                        'altitude_relative': 50 + math.sin(t * 0.2) * 5,
                        'bottom_clearance': 45 + math.sin(t * 0.2) * 5
                    },
                    'rc_channels': {
                        'chan1_raw': 1500 + int(math.sin(t) * 200),
                        'chan2_raw': 1500 + int(math.cos(t) * 200),
                        'chan3_raw': 1200 + int(math.sin(t * 0.5) * 300),
                        'chan4_raw': 1500 + int(math.sin(t * 0.7) * 200),
                        'rssi': 95
                    }
                }

                await self.send(json.dumps({
                    'type': 'telemetry',
                    'data': telemetry
                }))

                counter += 1
                await asyncio.sleep(0.1)

            except Exception as e:
                print(f"[SIMULATION] Error in simulated telemetry: {e}")
                break

    async def parameter_stream(self):
        await asyncio.sleep(1)
        counter = 0

        while self.running:
            try:
                parameters = {
                    'SURFACE_DEPTH': -10.0,
                    'FORMAT_VERSION': 1.0,
                    'SYSID_THISMAV': 1.0,
                    'BATT_CAPACITY': 3300.0,
                    'BATT_VOLT_MULT': 12.02,
                    'COMPASS_USE': 1.0,
                    'GPS_TYPE': 1.0,
                    'FRAME_CONFIG': 7.0,
                    'MOT_THST_HOVER': 0.5,
                    'ATC_ANG_RLL_P': 6.0,
                    'ATC_ANG_PIT_P': 6.0,
                    'ATC_ANG_YAW_P': 6.0,
                    'STAT_RUNTIME': 112712.0 + counter,
                    'ANGLE_MAX': 4500.0,
                    'SIMULATED': 1.0
                }

                await self.send(json.dumps({
                    'type': 'parameters',
                    'data': parameters
                }))

                counter += 1
                await asyncio.sleep(1)

            except Exception as e:
                print(f"[SIMULATION] Error in simulated parameters: {e}")
                break
