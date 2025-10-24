import serial
import re
import time
import csv
from datetime import datetime
import glob
import os

# ---------------- Clear existing CSV files ----------------
for csv_file in glob.glob("T*_rssi.csv") + glob.glob("receiver_positions.csv"):
    try:
        os.remove(csv_file)
        print(f"Cleared existing CSV file: {csv_file}")
    except Exception as e:
        print(f"Failed to delete {csv_file}: {e}")

# === Serial setup ===
SERIAL_PORT = "COM3"   # Change to your Arduino COM port
BAUD_RATE = 9600

# === KML output file ===
KML_FILE = "live_positions.kml"

# Dictionary for transmitters and receiver
transmitters = {}
receiver = None

# Regex patterns
tx_stored_pattern = re.compile(
    r"Stored\s+(T\d+)\s*\|\s*Lat:\s*([-0-9.]+)\s*\|\s*Lon:\s*([-0-9.]+)",
    re.IGNORECASE,
)

# 🔹 Filtered & Raw RSSI
tx_rssi_pattern = re.compile(r"^(T\d+)\s*\|\s*RSSI\s*:\s*(-?\d+\.?\d*)", re.IGNORECASE)
tx_rssi_raw_pattern = re.compile(r"^(T\d+)\s*\|\s*Raw RSSI\s*:\s*(-?\d+\.?\d*)", re.IGNORECASE)

tx_status_pattern = re.compile(r"Transmitter (T\d+) is (stale|active)\.", re.IGNORECASE)

# 🔹 Filtered & Raw receiver position
rx_pattern = re.compile(
    r"Estimated Receiver:.*Lat\s*=\s*([-0-9.]+)\s*\|\s*Lon\s*=\s*([-0-9.]+)",
    re.IGNORECASE,
)
rx_raw_pattern = re.compile(
    r"Raw Estimated Receiver:.*Lat\s*=\s*([-0-9.]+)\s*\|\s*Lon\s*=\s*([-0-9.]+)",
    re.IGNORECASE,
)

# ---------------- CSV logging function for RSSI ----------------
def log_rssi_csv(name, rssi, raw_rssi=None):
    """Append RSSI values (filtered & raw) with timestamp to a CSV file for each transmitter."""
    filename = f"{name}_rssi.csv"
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(filename, mode="a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(["Timestamp", "Filtered RSSI", "Raw RSSI"])
        writer.writerow([timestamp, rssi, raw_rssi])

# ---------------- CSV logging function for receiver ----------------
RECEIVER_CSV = "receiver_positions.csv"

def log_receiver_csv(lat, lon, raw_lat=None, raw_lon=None):
    """Append receiver position (filtered & raw) with timestamp to a CSV file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(RECEIVER_CSV, mode="a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(["Timestamp", "Latitude", "Longitude", "Raw Latitude", "Raw Longitude"])
        writer.writerow([timestamp, lat, lon, raw_lat, raw_lon])

# ---------------- KML writing function ----------------
def write_kml():
    """Write the KML file based on current transmitter states and receiver."""
    now = time.time()
    with open(KML_FILE, "w") as f:
        f.write('<?xml version="1.0" encoding="UTF-8"?>\n')
        f.write('<kml xmlns="http://www.opengis.net/kml/2.2">\n')
        f.write("<Document>\n")

        active_count = 0

        # Plot transmitters
        for name, info in transmitters.items():
            coords = info.get("coords")
            status = info.get("status", "stale")
            if coords is None:
                continue
            lat, lon = coords

            icon_url = (
                "http://maps.google.com/mapfiles/kml/paddle/red-circle.png"
                if status == "stale"
                else "http://maps.google.com/mapfiles/kml/paddle/blu-circle.png"
            )
            if status != "stale":
                active_count += 1

            f.write(f"""
            <Placemark>
                <name>{name}</name>
                <Style><IconStyle><scale>1.2</scale>
                <Icon><href>{icon_url}</href></Icon></IconStyle></Style>
                <Point><coordinates>{lon},{lat},0</coordinates></Point>
            </Placemark>
            """)

        # Plot receiver if available
        if receiver is not None:
            lat, lon = receiver
            if active_count >= 4:
                icon_url = "http://maps.google.com/mapfiles/kml/paddle/grn-circle.png"  # green
            elif active_count == 3:
                icon_url = "http://maps.google.com/mapfiles/kml/paddle/ylw-circle.png"  # yellow
            else:
                icon_url = "http://maps.google.com/mapfiles/kml/paddle/red-circle.png"  # red

            f.write(f"""
            <Placemark>
                <name>Receiver</name>
                <Style><IconStyle><scale>1.2</scale>
                <Icon><href>{icon_url}</href></Icon></IconStyle></Style>
                <Point><coordinates>{lon},{lat},0</coordinates></Point>
            </Placemark>
            """)

        f.write("</Document>\n</kml>\n")

# Write initial KML
write_kml()

# ---------------- Serial listening loop ----------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
print("Listening for Arduino data on", SERIAL_PORT)

# Buffers to hold raw values until filtered values are available
latest_raw_rssi = {}
latest_raw_position = (None, None)

while True:
    try:
        raw = ser.readline()
    except Exception as e:
        print("Serial read error:", e)
        time.sleep(0.5)
        continue

    if not raw:
        continue

    try:
        line = raw.decode("utf-8", errors="replace").strip()
    except Exception:
        line = raw.decode("latin1", errors="replace").strip()

    if not line:
        continue

    # -- 1) Stored line (initial transmitter coords)
    m = tx_stored_pattern.search(line)
    if m:
        name, lat_s, lon_s = m.groups()
        lat = float(lat_s)
        lon = float(lon_s)
        transmitters.setdefault(name, {})["coords"] = (lat, lon)
        transmitters[name]["status"] = "active"
        print(f"[STORED] {name} -> {lat}, {lon}")
        write_kml()
        continue

    # -- 2a) Raw RSSI line
    m_raw = tx_rssi_raw_pattern.search(line)
    if m_raw:
        name, raw_rssi_s = m_raw.groups()
        raw_rssi_val = float(raw_rssi_s)
        latest_raw_rssi[name] = raw_rssi_val
        print(f"[RAW RSSI] {name} -> {raw_rssi_val}")
        continue

    # -- 2b) Filtered RSSI line
    m2 = tx_rssi_pattern.search(line)
    if m2:
        name, rssi_val_s = m2.groups()
        rssi_val = float(rssi_val_s)
        raw_val = latest_raw_rssi.get(name)  # get last raw for this transmitter
        if name not in transmitters:
            transmitters[name] = {"coords": None, "status": "active"}
        transmitters[name]["status"] = "active"
        log_rssi_csv(name, rssi_val, raw_val)
        print(f"[RSSI] {name} -> Filtered={rssi_val}, Raw={raw_val}")
        write_kml()
        continue

    # -- 3) Status line
    m_status = tx_status_pattern.search(line)
    if m_status:
        name, status = m_status.groups()
        if name not in transmitters:
            transmitters[name] = {"coords": None}
        transmitters[name]["status"] = status.lower()
        print(f"[STATUS] {name} -> {status}")
        write_kml()
        continue

    # -- 4a) Raw Receiver position
    m3_raw = rx_raw_pattern.search(line)
    if m3_raw:
        lat_s, lon_s = m3_raw.groups()
        latest_raw_position = (float(lat_s), float(lon_s))
        print(f"[RAW RECEIVER] {latest_raw_position[0]}, {latest_raw_position[1]}")
        continue

    # -- 4b) Filtered Receiver estimated position
    m3 = rx_pattern.search(line)
    if m3:
        lat_s, lon_s = m3.groups()
        lat_val = float(lat_s)
        lon_val = float(lon_s)
        raw_lat, raw_lon = latest_raw_position
        receiver = (lat_val, lon_val)
        print(f"[RECEIVER] Filtered={receiver[0]}, {receiver[1]} | Raw={raw_lat}, {raw_lon}")
        log_receiver_csv(lat_val, lon_val, raw_lat, raw_lon)
        write_kml()
        continue
