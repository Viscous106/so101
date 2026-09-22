"""Read-only probe of the SO-101 Feetech bus. Writes NOTHING to the servos."""
import sys
import scservo_sdk as scs

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
NAMES = {1: "shoulder_pan", 2: "shoulder_lift", 3: "elbow_flex", 4: "wrist_flex", 5: "wrist_roll", 6: "gripper"}
# STS3215 control table (address, bytes)
REG = {"min_limit": (9, 2), "max_limit": (11, 2), "offset": (31, 2), "torque_enable": (40, 1), "goal_pos": (42, 2),
       "present_pos": (56, 2), "present_load": (60, 2), "voltage_x10": (62, 1), "temp_c": (63, 1), "moving": (66, 1)}

port = scs.PortHandler(PORT)
ph = scs.PacketHandler(0)
assert port.openPort(), f"cannot open {PORT}"
found = {}
for baud in (1_000_000, 500_000, 115_200):
    port.setBaudRate(baud)
    for i in range(1, 13):
        model, res, err = ph.ping(port, i)
        if res == scs.COMM_SUCCESS:
            found[i] = model
    if found:
        print(f"baud {baud}: found ids {sorted(found)}")
        break
else:
    print("no servos answered at any baud rate (is the 5V/12V supply on?)")
    sys.exit(1)

for i, model in sorted(found.items()):
    row = {}
    for k, (addr, n) in REG.items():
        val, res, err = (ph.read1ByteTxRx if n == 1 else ph.read2ByteTxRx)(port, i, addr)
        row[k] = val if res == scs.COMM_SUCCESS else None
    deg = None if row["present_pos"] is None else round((row["present_pos"] - 2048) * 360 / 4096, 1)
    print(f"id {i} {NAMES.get(i, '?'):14s} model={model} pos={row['present_pos']} (~{deg} deg from centre) "
          f"goal={row['goal_pos']} limits=[{row['min_limit']},{row['max_limit']}] offset={row['offset']} "
          f"torque={row['torque_enable']} load={row['present_load']} V={None if row['voltage_x10'] is None else row['voltage_x10']/10} "
          f"T={row['temp_c']}C moving={row['moving']}")
port.closePort()
