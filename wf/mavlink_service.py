
"""
MAVLink service for communicating with Pixhawk/ArduPilot
"""

from pymavlink import mavutil
import time


class MAVLinkService:
    def __init__(self, master):
        self.master = master
        self.connect()

    def connect(self):
        try:

            print("[PIXHAWK] Waiting for heartbeat...")
            self.master.wait_heartbeat()

            print(f"[PIXHAWK] Heartbeat from system {self.master.target_system}")
        except Exception as e:
            print(f"[CONNECTION] Failed to connect to Pixhawk: {e}")
            raise  # TODO: Error Handling UI

    def get_all_parameters(self):
        if not self.master:
            raise Exception("Not connected to Pixhawk")

        try:
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )

            parameters = {}
            timeout = time.time() + 10
            expected_count = None

            print("[PIXHAWK] Requesting parameters...")

            while True:
                if time.time() > timeout:
                    print(f"[PIXHAWK,IN] Timeout - Retrieved {len(parameters)} parameters")
                    break

                msg = self.master.recv_match(
                    type='PARAM_VALUE', blocking=True)

                if msg is None:
                    # If we have some parameters and no new messages, we're done
                    if len(parameters) > 0 and expected_count and len(parameters) >= expected_count:
                        break
                    continue

                param_id = msg.param_id
                # if isinstance(param_id, bytes):
                #     param_id = param_id.decode('utf-8')
                # param_id = param_id.strip('\x00')
                parameters[param_id] = msg.param_value

                if expected_count is None:
                    expected_count = msg.param_count
                    print(f"[PIXHAWK,IN] Expecting {expected_count} parameters")

                if msg.param_index + 1 >= msg.param_count:
                    print(f"[PIXHAWK,IN] Retrieved {len(parameters)} parameters")
                    break

            return parameters

        except Exception as e:
            print(f"[PIXHAWK,ERROR] Error getting parameters: {e}")
            return {}

    def get_parameter(self, param_name):
        """
        Get a specific parameter value
        """
        if not self.master:
            raise Exception("Not connected to Pixhawk")

        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode('utf-8'),
                -1
            )

            msg = self.master.recv_match(
                type='PARAM_VALUE', blocking=True)

            if msg:
                return msg.param_value

            return None

        except Exception as e:
            print(f"[PIXHAWK,ERROR] Error getting parameter {param_name}: {e}")
            return None

    def set_parameter(self, param_name, param_value):
        """
        Set a parameter value
        """
        if not self.master:
            raise Exception("Not connected to Pixhawk")

        try:
            print(f"[PIXHAWK,OUT] Setting {param_name} = {param_value}")

            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                param_name.encode('utf-8'),
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

            msg = self.master.recv_match(
                type='PARAM_VALUE', blocking=True)

            if msg:
                param_id = msg.param_id.decode('utf-8').strip('\x00')
                if param_id == param_name:
                    print(f"[PIXHAWK,OUT] Parameter {param_name} set to {msg.param_value}")
                    return True

            return False

        except Exception as e:
            print(f"[PIXHAWK,ERROR] Error setting parameter {param_name}: {e}")
            return False

    def get_telemetry_data(self):
        """
        Get current telemetry data
        """
        if not self.master:
            raise Exception("Not connected to Pixhawk")

        telemetry = {}

        try:
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                telemetry['roll'] = msg.roll
                telemetry['pitch'] = msg.pitch
                telemetry['yaw'] = msg.yaw

            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if msg:
                telemetry['latitude'] = msg.lat / 1e7
                telemetry['longitude'] = msg.lon / 1e7
                telemetry['altitude'] = msg.alt / 1000.0

            msg = self.master.recv_match(type='BATTERY_STATUS', blocking=False)
            if msg:
                telemetry['battery_voltage'] = msg.voltages[0] / 1000.0
                telemetry['battery_current'] = msg.current_battery / 100.0
                telemetry['battery_remaining'] = msg.battery_remaining

            return telemetry

        except Exception as e:
            print(f"[PIXHAWK,ERROR] Error getting telemetry: {e}")
            return {}

    def close(self):
        if self.master:
            self.master.close()
            self.master = None
            print("[CONNECTION] MAVLink connection closed")
