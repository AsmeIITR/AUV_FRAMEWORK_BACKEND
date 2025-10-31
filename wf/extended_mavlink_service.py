from pymavlink import mavutil
import time
from typing import Dict


class ExtendedMAVLinkService:

    def __init__(self, master):
        self.master = master
        self.connect()

    def connect(self):
        try:
            self.master.wait_heartbeat()
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                20,
                1
            )

            print(f"[PIXHAWK] Connected to system {self.master.target_system}")
        except Exception as e:
            print(f"[PIXHAWK,ERROR] Failed to connect: {e}")
            raise

    def get_attitude(self) -> Dict:
        """
        Get(roll, pitch, yaw)
        """
        msg = self.master.recv_match(
            type='ATTITUDE', blocking=True, timeout=0.5)
        if msg:
            return {
                'roll': msg.roll,
                'pitch': msg.pitch,
                'yaw': msg.yaw,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,   #
                'yawspeed': msg.yawspeed,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_attitude_quaternion(self) -> Dict:
        msg = self.master.recv_match(
            type='ATTITUDE_QUATERNION', blocking=True, timeout=0.5)
        if msg:
            return {
                'q1': msg.q1,
                'q2': msg.q2,
                'q3': msg.q3,
                'q4': msg.q4,
                'rollspeed': msg.rollspeed,
                'pitchspeed': msg.pitchspeed,
                'yawspeed': msg.yawspeed,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_gps_raw(self) -> Dict:
        msg = self.master.recv_match(
            type='GPS_RAW_INT', blocking=True, timeout=0.5)
        if msg:
            return {
                'fix_type': msg.fix_type,
                'latitude': msg.lat / 1e7,
                'longitude': msg.lon / 1e7,
                'altitude': msg.alt / 1000.0,
                'eph': msg.eph / 100.0,
                'epv': msg.epv / 100.0,
                'velocity': msg.vel / 100.0,
                # Course over ground (degrees)
                'cog': msg.cog / 100.0,
                'satellites_visible': msg.satellites_visible,
                'timestamp': msg.time_usec
            }
        return {}

    def get_global_position(self) -> Dict:
        msg = self.master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
        if msg:
            return {
                'latitude': msg.lat / 1e7,
                'longitude': msg.lon / 1e7,
                'altitude_msl': msg.alt / 1000.0,
                'altitude_rel': msg.relative_alt / 1000.0,
                'vx': msg.vx / 100.0,
                'vy': msg.vy / 100.0,
                'vz': msg.vz / 100.0,
                'heading': msg.hdg / 100.0,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_local_position(self) -> Dict:
        msg = self.master.recv_match(
            type='LOCAL_POSITION_NED', blocking=True, timeout=0.5)
        if msg:
            return {
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'vx': msg.vx,
                'vy': msg.vy,
                'vz': msg.vz,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_battery_status(self) -> Dict:
        msg = self.master.recv_match(
            type='BATTERY_STATUS', blocking=True, timeout=0.5)
        if msg:
            return {
                'id': msg.id,
                'battery_function': msg.battery_function,
                'type': msg.type,
                'temperature': msg.temperature / 100.0,
                'voltages': [v / 1000.0 for v in msg.voltages if v != 65535],
                'current_battery': msg.current_battery / 100.0,
                'current_consumed': msg.current_consumed,
                'energy_consumed': msg.energy_consumed,
                'battery_remaining': msg.battery_remaining,
                'time_remaining': msg.time_remaining,
                'charge_state': msg.charge_state
            }
        return {}

    def get_sys_status(self) -> Dict:
        msg = self.master.recv_match(
            type='SYS_STATUS', blocking=True, timeout=0.5)
        if msg:
            return {
                'voltage_battery': msg.voltage_battery / 1000.0,
                'current_battery': msg.current_battery / 100.0,
                'battery_remaining': msg.battery_remaining,
                'drop_rate_comm': msg.drop_rate_comm,
                'errors_comm': msg.errors_comm,
                'errors_count1': msg.errors_count1,
                'errors_count2': msg.errors_count2,
                'errors_count3': msg.errors_count3,
                'errors_count4': msg.errors_count4,
                'onboard_control_sensors_present': msg.onboard_control_sensors_present,
                'onboard_control_sensors_enabled': msg.onboard_control_sensors_enabled,
                'onboard_control_sensors_health': msg.onboard_control_sensors_health
            }
        return {}

    def get_raw_imu(self) -> Dict:
        msg = self.master.recv_match(
            type='RAW_IMU', blocking=True, timeout=0.5)
        if msg:
            return {
                'xacc': msg.xacc,
                'yacc': msg.yacc,
                'zacc': msg.zacc,
                'xgyro': msg.xgyro,
                'ygyro': msg.ygyro,
                'zgyro': msg.zgyro,
                'xmag': msg.xmag,
                'ymag': msg.ymag,
                'zmag': msg.zmag,
                'timestamp': msg.time_usec
            }
        return {}

    def get_scaled_imu(self) -> Dict:
        msg = self.master.recv_match(
            type='SCALED_IMU', blocking=True, timeout=0.5)
        if msg:
            return {
                'xacc': msg.xacc / 1000.0,
                'yacc': msg.yacc / 1000.0,
                'zacc': msg.zacc / 1000.0,
                'xgyro': msg.xgyro / 1000.0,
                'ygyro': msg.ygyro / 1000.0,
                'zgyro': msg.zgyro / 1000.0,
                'xmag': msg.xmag / 1000.0,
                'ymag': msg.ymag / 1000.0,
                'zmag': msg.zmag / 1000.0,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_scaled_pressure(self) -> Dict:
        msg = self.master.recv_match(
            type='SCALED_PRESSURE', blocking=True, timeout=0.5)
        if msg:
            return {
                'press_abs': msg.press_abs,
                'press_diff': msg.press_diff,
                'temperature': msg.temperature / 100.0,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_altitude(self) -> Dict:
        msg = self.master.recv_match(
            type='ALTITUDE', blocking=True, timeout=0.5)
        if msg:
            return {
                'altitude_monotonic': msg.altitude_monotonic,
                'altitude_amsl': msg.altitude_amsl,
                'altitude_local': msg.altitude_local,
                'altitude_relative': msg.altitude_relative,
                'altitude_terrain': msg.altitude_terrain,
                'bottom_clearance': msg.bottom_clearance,
                'timestamp': msg.time_usec
            }
        return {}

    def get_vfr_hud(self) -> Dict:
        """
        Get HUD (Heads-Up Display) data
        Message: VFR_HUD (#74)
        """
        msg = self.master.recv_match(
            type='VFR_HUD', blocking=True, timeout=0.5)
        if msg:
            return {
                'airspeed': msg.airspeed,
                'groundspeed': msg.groundspeed,
                'heading': msg.heading,
                'throttle': msg.throttle,
                'alt': msg.alt,
                'climb': msg.climb
            }
        return {}

    def get_rc_channels(self) -> Dict:
        msg = self.master.recv_match(
            type='RC_CHANNELS', blocking=True, timeout=0.5)
        if msg:
            return {
                'chan1_raw': msg.chan1_raw,
                'chan2_raw': msg.chan2_raw,
                'chan3_raw': msg.chan3_raw,
                'chan4_raw': msg.chan4_raw,
                'chan5_raw': msg.chan5_raw,
                'chan6_raw': msg.chan6_raw,
                'chan7_raw': msg.chan7_raw,
                'chan8_raw': msg.chan8_raw,
                'chan9_raw': msg.chan9_raw,
                'chan10_raw': msg.chan10_raw,
                'chan11_raw': msg.chan11_raw,
                'chan12_raw': msg.chan12_raw,
                'chan13_raw': msg.chan13_raw,
                'chan14_raw': msg.chan14_raw,
                'chan15_raw': msg.chan15_raw,
                'chan16_raw': msg.chan16_raw,
                'chan17_raw': msg.chan17_raw,
                'chan18_raw': msg.chan18_raw,
                'chancount': msg.chancount,
                'rssi': msg.rssi,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_servo_output(self) -> Dict:
        msg = self.master.recv_match(
            type='SERVO_OUTPUT_RAW', blocking=True, timeout=0.5)
        if msg:
            return {
                'servo1_raw': msg.servo1_raw,
                'servo2_raw': msg.servo2_raw,
                'servo3_raw': msg.servo3_raw,
                'servo4_raw': msg.servo4_raw,
                'servo5_raw': msg.servo5_raw,
                'servo6_raw': msg.servo6_raw,
                'servo7_raw': msg.servo7_raw,
                'servo8_raw': msg.servo8_raw,
                'servo9_raw': msg.servo9_raw,
                'servo10_raw': msg.servo10_raw,
                'servo11_raw': msg.servo11_raw,
                'servo12_raw': msg.servo12_raw,
                'servo13_raw': msg.servo13_raw,
                'servo14_raw': msg.servo14_raw,
                'servo15_raw': msg.servo15_raw,
                'servo16_raw': msg.servo16_raw,
                'port': msg.port,
                'timestamp': msg.time_usec
            }
        return {}

    def get_mission_current(self) -> Dict:
        msg = self.master.recv_match(
            type='MISSION_CURRENT', blocking=True, timeout=0.5)
        if msg:
            return {
                'seq': msg.seq,
                'total': msg.total if hasattr(msg, 'total') else None,
                'mission_state': msg.mission_state if hasattr(msg, 'mission_state') else None,
                'mission_mode': msg.mission_mode if hasattr(msg, 'mission_mode') else None
            }
        return {}

    def get_nav_controller_output(self) -> Dict:
        msg = self.master.recv_match(
            type='NAV_CONTROLLER_OUTPUT', blocking=True, timeout=0.5)
        if msg:
            return {
                'nav_roll': msg.nav_roll,
                'nav_pitch': msg.nav_pitch,
                'nav_bearing': msg.nav_bearing,
                'target_bearing': msg.target_bearing,
                'wp_dist': msg.wp_dist,
                'alt_error': msg.alt_error,
                'aspd_error': msg.aspd_error,
                'xtrack_error': msg.xtrack_error
            }
        return {}

    def get_heartbeat(self) -> Dict:
        msg = self.master.recv_match(
            type='HEARTBEAT', blocking=True, timeout=0.5)
        if msg:
            return {
                'type': msg.type,
                'autopilot': msg.autopilot,
                'base_mode': msg.base_mode,
                'custom_mode': msg.custom_mode,
                'system_status': msg.system_status,
                'mavlink_version': msg.mavlink_version
            }
        return {}

    def get_rangefinder(self) -> Dict:
        msg = self.master.recv_match(
            type='DISTANCE_SENSOR', blocking=True, timeout=0.5)
        if msg:
            return {
                'min_distance': msg.min_distance / 100.0,
                'max_distance': msg.max_distance / 100.0,
                'current_distance': msg.current_distance / 100.0,
                'type': msg.type,
                'id': msg.id,
                'orientation': msg.orientation,
                'covariance': msg.covariance,
                'timestamp': msg.time_boot_ms
            }
        return {}

    def get_optical_flow(self) -> Dict:
        """
        Get optical flow sensor data
        Message: OPTICAL_FLOW (#100)
        """
        msg = self.master.recv_match(
            type='OPTICAL_FLOW', blocking=True, timeout=0.5)
        if msg:
            return {
                'flow_x': msg.flow_x,
                'flow_y': msg.flow_y,
                'flow_comp_m_x': msg.flow_comp_m_x,
                'flow_comp_m_y': msg.flow_comp_m_y,
                'quality': msg.quality,
                'ground_distance': msg.ground_distance,
                'timestamp': msg.time_usec
            }
        return {}

    def get_wind(self) -> Dict:
        msg = self.master.recv_match(type='WIND', blocking=False)
        if msg:
            return {
                'direction': msg.direction,
                'speed': msg.speed,
                'speed_z': msg.speed_z
            }
        return {}

    def get_all_telemetry(self) -> Dict:
        telemetry = {
            'timestamp': time.time(),
            'attitude': self.get_attitude(),
            'gps': self.get_gps_raw(),
            'global_position': self.get_global_position(),
            'local_position': self.get_local_position(),
            'battery': self.get_battery_status(),
            'sys_status': self.get_sys_status(),
            'raw_imu': self.get_raw_imu(),
            'scaled_imu': self.get_scaled_imu(),
            'pressure': self.get_scaled_pressure(),
            'altitude': self.get_altitude(),
            'vfr_hud': self.get_vfr_hud(),
            'rc_channels': self.get_rc_channels(),
            'servo_output': self.get_servo_output(),
            'mission': self.get_mission_current(),
            'nav_controller': self.get_nav_controller_output(),
            'heartbeat': self.get_heartbeat(),
            'rangefinder': self.get_rangefinder(),
        }

        return telemetry

    def close(self):
        if self.master:
            self.master.close()
            self.master = None
            print("[CONNECTION] AVLink connection closed")
