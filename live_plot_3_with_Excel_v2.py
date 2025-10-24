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
tx_rssi_pattern = re.compile(r"^(T\d+)\s*\|\s*RSSI\s*:\s*(-?\d+\.?\d*)", re.IGNORECASE)
tx_status_pattern = re.compile(r"Transmitter (T\d+) is (stale|active)\.", re.IGNORECASE)
rx_pattern = re.compile(
    r"Estimated Receiver:.*Lat\s*=\s*([-0-9.]+)\s*\|\s*Lon\s*=\s*([-0-9.]+)",
    re.IGNORECASE,
)

# ---------------- CSV logging function for RSSI ----------------
def log_rssi_csv(name, rssi):
    """Append RSSI value with timestamp to a CSV file for each transmitter."""
    filename = f"{name}_rssi.csv"
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(filename, mode="a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(["Timestamp", "RSSI"])
        writer.writerow([timestamp, rssi])

# ---------------- CSV logging function for receiver ----------------
RECEIVER_CSV = "receiver_positions.csv"

def log_receiver_csv(lat, lon):
    """Append receiver position with timestamp to a CSV file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(RECEIVER_CSV, mode="a", newline="") as f:
        writer = csv.writer(f)
        if f.tell() == 0:
            writer.writerow(["Timestamp", "Latitude", "Longitude"])
        writer.writerow([timestamp, lat, lon])

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

    # -- 2) RSSI line
    m2 = tx_rssi_pattern.search(line)
    if m2:
        name, rssi_val_s = m2.groups()
        rssi_val = float(rssi_val_s)
        if name not in transmitters:
            transmitters[name] = {"coords": None, "status": "active"}
        transmitters[name]["status"] = "active"
        log_rssi_csv(name, rssi_val)
        print(f"[RSSI] {name} -> {rssi_val}")
        write_kml()
        continue

    # -- 3) Status line ("T1 is stale" / "T1 is active")
    m_status = tx_status_pattern.search(line)
    if m_status:
        name, status = m_status.groups()
        if name not in transmitters:
            transmitters[name] = {"coords": None}
        transmitters[name]["status"] = status.lower()
        print(f"[STATUS] {name} -> {status}")
        write_kml()
        continue

    # -- 4) Receiver estimated position
    m3 = rx_pattern.search(line)
    if m3:
        lat_s, lon_s = m3.groups()
        lat_val = float(lat_s)
        lon_val = float(lon_s)
        receiver = (lat_val, lon_val)
        print(f"[RECEIVER] {receiver[0]}, {receiver[1]}")
        # Log to CSV
        log_receiver_csv(lat_val, lon_val)
        write_kml()
        continue
