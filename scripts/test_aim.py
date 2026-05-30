"""
test_aim.py — test directo de la balistica del mock.

Agente sniper estatico que apunta y dispara contra un blanco fijo.
Si no pega, hay bug en mock o Ballistic.py.
"""
import math, socket, sys, time, struct
sys.path.insert(0, 'scripts')
from Ballistic import BallisticTable, solve_moving_intercept, world_bearing_to_turret, SIM_DT
from TelemetryDictionary import telemetrydirs as td
from Command import Command
import Configuration

if len(sys.argv) < 2:
    print("uso: python3 scripts/test_aim.py {1|2}")
    sys.exit(1)
tank_id = int(sys.argv[1])

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', 4600 + tank_id))
sock.settimeout(10)
command = Command(Configuration.ip, 4500 + tank_id)
table = BallisticTable()
TELEMETRY_STRUCT = '<LLififffffffffffffffffff'
TELEMETRY_LEN = 96
FIRE = 11

print(f"test_aim tank {tank_id} listening 460{tank_id}")

shots_fired = 0
while shots_fired < 20:
    try:
        data, _ = sock.recvfrom(TELEMETRY_LEN)
    except socket.timeout:
        print("timeout"); break
    if len(data) != TELEMETRY_LEN: continue
    v = struct.unpack(TELEMETRY_STRUCT, data)
    n = int(v[td['number']])
    if n != tank_id:
        # leer del otro tanque
        try:
            data2, _ = sock.recvfrom(TELEMETRY_LEN)
            v2 = struct.unpack(TELEMETRY_STRUCT, data2)
            if int(v2[td['number']]) == (3 - tank_id):
                mine, other = v, v2
            else:
                continue
        except socket.timeout:
            continue
    else:
        try:
            data2, _ = sock.recvfrom(TELEMETRY_LEN)
            v2 = struct.unpack(TELEMETRY_STRUCT, data2)
            if int(v2[td['number']]) == (3 - tank_id):
                mine, other = v, v2
            else:
                continue
        except socket.timeout:
            continue

    my_x = float(mine[td['x']]); my_z = float(mine[td['z']]); my_y = float(mine[td['y']])
    my_az = float(mine[td['azimuth']])
    ox = float(other[td['x']]); oz = float(other[td['z']]); oy = float(other[td['y']])
    timer = int(mine[td['timer']])
    pw = int(mine[td['power']])

    # rival estatico => vel=0
    intercept = solve_moving_intercept((my_x, my_z), (ox, oz), (0.0, 0.0), table,
                                       shooter_y=my_y, target_y=oy)
    if intercept is None:
        print(f"t={timer} no intercept (dist?)")
        continue

    decl = intercept['decl_deg']
    wb = intercept['bearing_world_deg']
    tb = world_bearing_to_turret(wb, my_az)
    dist = math.hypot(ox-my_x, oz-my_z)

    print(f"t={timer:>5} dist={dist:6.1f}m  my=({my_x:.0f},{my_z:.0f}) other=({ox:.0f},{oz:.0f})  "
          f"decl={decl:+.3f} world_bearing={wb:+.1f} turret_bearing(rel chassis)={tb:+.1f} az={my_az:+.1f}")

    # frenar, apuntar, disparar
    command.command = FIRE
    command.send_command(timer, tank_id, 0.0, 0.0, decl, tb)
    shots_fired += 1
    time.sleep(0.3)
print(f"disparados {shots_fired}")
