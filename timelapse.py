import os
import time
import subprocess
from pymavlink import mavutil
import datetime
import piexif

def timelapse(master,mode_check,interval,base_dir):
    seq = 1
    dt, session_dir = create_datadir(base_dir)
    pstr = f'Geotagged images will be saved to {session_dir}'
    print(pstr)
    log_name = dt.strftime("%Y%m%d_%H%M%S_img_log.csv")
    log_path = os.path.join(session_dir,log_name)
    pstr = f'Log file: {log_path}'
    print(pstr)
    f = open(log_path,'w+')
    starttime = datetime.datetime.now()
    while True:
        mode = get_mode(master)
        armed = get_armed(master)
        if mode != mode_check or armed == 0:
            pstr = 'Mode or arming state changed. Stopping timelapse. Closing log file'
            print(pstr)
            f.close()
            break
        else:
            dt = datetime.datetime.now()
            elapsed = (dt - starttime).total_seconds()
            if elapsed >= interval:
                starttime = datetime.datetime.now()
                lat,lon = get_gps(master)
                fpath = grab_still_gps(seq,session_dir,lat,lon,dt)
                pstr = f'image {fpath} acquired at {lat}, {lon}'
                print(pstr)
                pstr = f'{fpath},{lat},{lon},{dt.strftime("%Y-%m-%d %H%M%S")}\n'
                f.write(pstr)
                seq += 1

def create_datadir(base_dir):
    dt = datetime.datetime.now()
    name = dt.strftime("%Y%m%d_%H%M%S_UTC")
    session_dir = os.path.join(base_dir, name)
    os.makedirs(session_dir, exist_ok=True)
    return dt, session_dir

def decimal_to_dms_rational(value):
    v = abs(value)
    deg = int(v)
    rem = (v - deg) * 60.0
    minute = int(rem)
    sec = (rem - minute) * 60.0
    return ((deg,1),(minute,1),(int(sec*1000),1000))

def write_exif_with_piexif(filepath, lat, lon, dt):
    gps_ifd = {}
    gps_ifd[piexif.GPSIFD.GPSLatitudeRef]  = b'N' if lat >= 0 else b'S'
    gps_ifd[piexif.GPSIFD.GPSLatitude]     = decimal_to_dms_rational(lat)
    gps_ifd[piexif.GPSIFD.GPSLongitudeRef] = b'E' if lon >= 0 else b'W'
    gps_ifd[piexif.GPSIFD.GPSLongitude]    = decimal_to_dms_rational(lon)
    dt_str = dt.strftime("%Y:%m:%d %H:%M:%S")
    exif_ifd = {piexif.ExifIFD.DateTimeOriginal: dt_str.encode()}
    exif_dict = {"0th": {}, "Exif": exif_ifd, "GPS": gps_ifd, "1st": {}, "thumbnail": None}
    piexif.insert(piexif.dump(exif_dict), filepath)

def grab_still_gps(seq,session_dir,lat,lon,dt):
    device = "/dev/video0"
    w = 1920
    h = 1080
    fname = f"img_{seq:04d}.jpg"
    fpath = os.path.join(session_dir, fname)
    # Capture single frame with ffmpeg
    ffmpeg_cmd = [
        "ffmpeg",
        "-y",
        "-f", "v4l2",
        "-input_format", "mjpeg",
        "-video_size", f"{w}x{h}",
        "-i", device,
        "-frames:v", "1",
        fpath
    ]
    subprocess.run(ffmpeg_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    write_exif_with_piexif(fpath, lat, lon, dt)
    return fpath

def get_armed(master):
    try:
        armed = master.motors_armed()
        return armed
    except Exception as e:
        print(e)
        return None

def get_mode(master):
    good_mode = False
    mode = None
    while not good_mode:
        msg = master.recv_match(type = 'HEARTBEAT',blocking=False)
        if msg is not None:
            mode = mavutil.mode_string_v10(msg)
            if mode is not None and 'x' not in mode:
                good_mode = True
    return mode

def get_gps(master):
    lat = None
    lon = None
    latcheck = False
    while not latcheck:
        # Try GLOBAL_POSITION_INT first
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg:
            lat = msg.lat / 1e7
            latcheck = True
            lon = msg.lon / 1e7
            pstr = "Messages received"
            print(pstr)
            pstr = f"[GLOBAL_POSITION_INT] lat={lat:.7f}, lon={lon:.7f}"
            print(pstr)
        if lat is None:
            # Fallback to GPS_RAW_INT
            msg = master.recv_match(type="GPS_RAW_INT", blocking=False)
            if msg:
                lat = msg.lat / 1e7
                latcheck = True
                lon = msg.lon / 1e7
                pstr = "Messages received"
                print(pstr)
                pstr = f"[GPS_RAW_INT] lat={lat:.7f}, lon={lon:.7f}"
                print(pstr)
        time.sleep(0.1)
    return lat,lon

# constants - don't change these
MAVLINK_ENDPOINT = "tcp:127.0.0.1:5777"  # change if needed

# user-defined
delay_start_condition = False # delay the start of the script - to acquire GPS, etc.
delay_start_time = 120 # time to wait before starting the script (seconds)
seq = 1
interval = 2
mode_check = 'AUTO'
base_dir = "/home/pi/timelapse"


# wait time - optional
if delay_start_condition:
    pstr = f'Start delayed. Waiting {delay_start_time} seconds.'
    print(pstr)
    time.sleep(delay_start_time)

print("Connecting to MAVLink...")
master = mavutil.mavlink_connection(MAVLINK_ENDPOINT)
master.wait_heartbeat(timeout=5)
print(f"Connected to system {master.target_system}, component {master.target_component}")

# check GPS
pstr = 'Checking GPS'
print(pstr)
gpscheck = False
lat,lon = get_gps(master)

pstr = f'Lat: {lat}. Lon: {lon}'
print(pstr)

if lat is not None:
    gpscheck = True
    pstr = f'MAVlink GPS test passed. Lat: {lat}. Lon: {lon}'
    print(pstr)

if gpscheck:
    pstr = 'Checking vehicle mode and arming state'
    print(pstr)
    try:
        while True:
            mode = get_mode(master)
            armed = get_armed(master)
            if mode == mode_check and armed > 0:
                pstr = f'Vehicle in {mode_check} and armed. Starting timelapse'
                timelapse(master, mode_check, interval, base_dir)
            else:
                pstr = f'Vehicle in {mode} and disarmed. Waiting'

            print(pstr)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pstr = 'Exiting loop ...'
        print(pstr)
