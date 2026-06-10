import socket
import json
import math
import random
import time
import threading
import struct

MU = 398600.4418
RE = 6378.137
J2 = 1.08263e-3

NUM_SATS = 80
NUM_PLANES = 5
SATS_PER_PLANE = 16
BASE_SMA = RE + 550.0
INCLINATION_RAD = math.radians(53.0)
RAAN_SPACING = math.radians(72.0)
TA_SPACING = math.radians(22.5)

TELEMETRY_INTERVAL = 30.0
TLE_INTERVAL = 300.0
UDP_HOST = "127.0.0.1"
TELEMETRY_PORT = 9090
TLE_PORT = 9091
NUM_THREADS = 10
SATS_PER_THREAD = NUM_SATS // NUM_THREADS

DRAG_RATE = 0.01 / 86400.0
PROPELLANT_RATE = 0.001


def mean_motion_rad(a):
    return math.sqrt(MU / a ** 3)


def mean_motion_revday(a):
    return mean_motion_rad(a) * 86400.0 / (2.0 * math.pi)


def true_to_mean_anomaly(nu, e):
    e2 = 2.0 * math.atan2(
        math.sqrt(1.0 - e) * math.sin(nu / 2.0),
        math.sqrt(1.0 + e) * math.cos(nu / 2.0),
    )
    return e2 - e * math.sin(e2)


def orbital_to_eci(a, e, i, raan, omega, nu):
    p = a * (1.0 - e * e)
    r = p / (1.0 + e * math.cos(nu))
    xp = r * math.cos(nu)
    yp = r * math.sin(nu)
    h = math.sqrt(MU * p)
    vxp = -MU / h * math.sin(nu)
    vyp = MU / h * (e + math.cos(nu))
    co, so = math.cos(omega), math.sin(omega)
    cO, sO = math.cos(raan), math.sin(raan)
    ci, si = math.cos(i), math.sin(i)
    r11 = cO * co - sO * so * ci
    r12 = -cO * so - sO * co * ci
    r21 = sO * co + cO * so * ci
    r22 = -sO * so + cO * co * ci
    r31 = so * si
    r32 = co * si
    pos = (r11 * xp + r12 * yp, r21 * xp + r22 * yp, r31 * xp + r32 * yp)
    vel = (r11 * vxp + r12 * vyp, r21 * vxp + r22 * vyp, r31 * vxp + r32 * vyp)
    return pos, vel


def j2_raan_drift(a, e, i):
    n = mean_motion_rad(a)
    return -1.5 * n * J2 * (RE / a) ** 2 * math.cos(i) / (1.0 - e * e) ** 2


def tle_checksum(line):
    total = 0
    for c in line:
        if c.isdigit():
            total += int(c)
        elif c == "-":
            total += 1
    return total % 10


def bstar_to_tle_str(bstar):
    if bstar == 0:
        return "+00000-0"
    sign = "+" if bstar > 0 else "-"
    ab = abs(bstar)
    exp = math.floor(math.log10(ab))
    mantissa = ab / 10.0 ** (exp + 1)
    digits = int(round(mantissa * 100000))
    if digits >= 100000:
        digits = 99999
    return "{}{:05d}{:+01d}".format(sign, digits, exp + 1)


def generate_tle(sat):
    now = time.time()
    t = time.gmtime(now)
    ey = t.tm_year % 100
    ed = t.tm_yday + (t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec) / 86400.0
    nid = sat["norad_id"]
    inc = math.degrees(sat["inclination"])
    raan = math.degrees(sat["raan"]) % 360.0
    ecc = sat["eccentricity"]
    argp = math.degrees(sat["arg_perigee"]) % 360.0
    ma = math.degrees(sat["mean_anomaly"]) % 360.0
    mm = mean_motion_revday(sat["semi_major_axis"])
    ecc_str = "{:07d}".format(int(round(ecc * 1e7)))
    bs = bstar_to_tle_str(sat["bstar"])
    intl = "{:02d}{:03d}A".format(ey, t.tm_yday)
    line1_body = "1 {:05d}U {:8s} {:02d}{:012.8f}  .00000000  00000-0  {:8s} 0  9999".format(
        nid, intl, ey, ed, bs
    )
    line2_body = "2 {:05d} {:8.4f} {:8.4f} {:7s} {:8.4f} {:8.4f} {:11.8f}{:05d}".format(
        nid, inc, raan, ecc_str, argp, ma, mm, 0
    )
    line1 = line1_body[:68] + str(tle_checksum(line1_body[:68]))
    line2 = line2_body[:68] + str(tle_checksum(line2_body[:68]))
    return line1, line2


def init_satellite(sat_id):
    plane = (sat_id - 1) // SATS_PER_PLANE
    index = (sat_id - 1) % SATS_PER_PLANE
    sma = BASE_SMA + random.uniform(-5, 5)
    ecc = 0.001 + random.uniform(-0.0005, 0.0005)
    nu = index * TA_SPACING + random.uniform(-0.01, 0.01)
    return {
        "satellite_id": sat_id,
        "norad_id": 40001 + sat_id,
        "plane": plane,
        "semi_major_axis": sma,
        "eccentricity": ecc,
        "inclination": INCLINATION_RAD,
        "raan": plane * RAAN_SPACING,
        "arg_perigee": 0.0,
        "true_anomaly": nu,
        "propellant": 50.0 + random.uniform(-5, 5),
        "quat_w": 1.0,
        "quat_x": 0.0,
        "quat_y": 0.0,
        "quat_z": 0.0,
        "quat_phase": 0.0,
        "bstar": random.uniform(0.0001, 0.001),
        "mean_anomaly": true_to_mean_anomaly(nu, ecc),
    }


def update_satellite(sat, dt):
    n = mean_motion_rad(sat["semi_major_axis"])
    sat["true_anomaly"] = (sat["true_anomaly"] + n * dt) % (2.0 * math.pi)
    sat["semi_major_axis"] -= DRAG_RATE * dt
    raan_drift = j2_raan_drift(sat["semi_major_axis"], sat["eccentricity"], sat["inclination"])
    sat["raan"] = (sat["raan"] + raan_drift * dt) % (2.0 * math.pi)
    sat["propellant"] -= PROPELLANT_RATE * (1.0 + random.uniform(-0.1, 0.1))
    sat["propellant"] = max(0.0, sat["propellant"])
    sat["quat_phase"] = (sat["quat_phase"] + 0.001 * dt) % (2.0 * math.pi)
    sat["quat_w"] = math.cos(sat["quat_phase"])
    sat["quat_x"] = 0.0
    sat["quat_y"] = 0.0
    sat["quat_z"] = math.sin(sat["quat_phase"])
    sat["mean_anomaly"] = true_to_mean_anomaly(sat["true_anomaly"], sat["eccentricity"])


def build_telemetry(sat):
    pos, vel = orbital_to_eci(
        sat["semi_major_axis"],
        sat["eccentricity"],
        sat["inclination"],
        sat["raan"],
        sat["arg_perigee"],
        sat["true_anomaly"],
    )
    return json.dumps(
        {
            "satellite_id": sat["satellite_id"],
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "semi_major_axis": sat["semi_major_axis"],
            "eccentricity": sat["eccentricity"],
            "inclination": sat["inclination"],
            "raan": sat["raan"],
            "arg_perigee": sat["arg_perigee"],
            "true_anomaly": sat["true_anomaly"],
            "quat_w": sat["quat_w"],
            "quat_x": sat["quat_x"],
            "quat_y": sat["quat_y"],
            "quat_z": sat["quat_z"],
            "propellant_remaining": sat["propellant"],
            "position_x": pos[0],
            "position_y": pos[1],
            "position_z": pos[2],
            "velocity_x": vel[0],
            "velocity_y": vel[1],
            "velocity_z": vel[2],
        }
    )


def worker(satellites, tel_sock, tle_sock, stop_event):
    last_tle_time = time.time()
    while not stop_event.is_set():
        cycle_start = time.time()
        for sat in satellites:
            update_satellite(sat, TELEMETRY_INTERVAL)
            msg = build_telemetry(sat).encode("utf-8")
            try:
                tel_sock.sendto(msg, (UDP_HOST, TELEMETRY_PORT))
            except OSError:
                pass
        now = time.time()
        if now - last_tle_time >= TLE_INTERVAL:
            for sat in satellites:
                l1, l2 = generate_tle(sat)
                tle_msg = "{}\n{}".format(l1, l2).encode("utf-8")
                try:
                    tle_sock.sendto(tle_msg, (UDP_HOST, TLE_PORT))
                except OSError:
                    pass
            last_tle_time = now
        elapsed = time.time() - cycle_start
        sleep_time = max(0, TELEMETRY_INTERVAL - elapsed)
        if stop_event.wait(sleep_time):
            break


def main():
    satellites = [init_satellite(i + 1) for i in range(NUM_SATS)]
    tel_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tle_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    stop_event = threading.Event()
    threads = []
    for t_idx in range(NUM_THREADS):
        start = t_idx * SATS_PER_THREAD
        end = start + SATS_PER_THREAD
        group = satellites[start:end]
        th = threading.Thread(
            target=worker,
            args=(group, tel_sock, tle_sock, stop_event),
            daemon=True,
        )
        threads.append(th)
        th.start()
    print(
        "Constellation simulator started: {} satellites, {} threads".format(
            NUM_SATS, NUM_THREADS
        )
    )
    print(
        "Telemetry -> {}:{} | TLE -> {}:{}".format(
            UDP_HOST, TELEMETRY_PORT, UDP_HOST, TLE_PORT
        )
    )
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        stop_event.set()
        for th in threads:
            th.join(timeout=5)
    finally:
        tel_sock.close()
        tle_sock.close()
        print("Simulator stopped.")


if __name__ == "__main__":
    main()
