from pymavlink import mavutil
from datetime import datetime, timedelta, timezone
import time

def get_gps(master):
    """Retrieve GPS position and GPS time (from GPS week + ms) for a specific GPS ID."""
    lat, lon, gps_time = None, None, None

    while True:
        msg = master.recv_match(type=["GPS_RAW_INT","GPS2_RAW"], blocking=False)

        if (
            msg
            and msg.lat > 0
        ):
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            msg2 = master.recv_match(type="SYSTEM_TIME", blocking=False)
            if msg2 and msg2.time_unix_usec > 0:
                gps_time = datetime.fromtimestamp(msg2.time_unix_usec / 1e6, tz=timezone.utc)
                dt = datetime.now()
                print(f"[GPS_RAW_INT id={msg.id}] lat={lat:.7f}, lon={lon:.7f}, time:{gps_time}, datetime.now:{dt.strftime('%Y-%m-%d %H%M%S')}")
                break

        time.sleep(0.1)

    return lat, lon, gps_time


MAVLINK_ENDPOINT = "tcp:127.0.0.1:5777"  # change if needed

print("Connecting to MAVLink...")
master = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
master.wait_heartbeat(timeout=5)
print(f"Connected to system {master.target_system}, component {master.target_component}")

try:
    while True:
        lat, lon, gps_time = get_gps(master)
except KeyboardInterrupt:
    pstr = 'Exiting loop ...'
    print(pstr)