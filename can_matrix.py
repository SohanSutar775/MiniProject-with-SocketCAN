"""CAN matrix, signal encoding and plausibility limits for the Vehicle Monitoring System (VMS)."""
import struct

BITRATE = 500_000  # bit/s

# ---- Message IDs (11-bit, lower value = higher priority) ----
ID_MOTION   = 0x100   # Sensor ECU     -> Controller, Diagnostic  (speed, rpm)         periodic 20 ms
ID_COOLANT  = 0x101   # Sensor ECU     -> Controller, Diagnostic  (coolant temp)       periodic 100 ms
ID_BATTERY  = 0x102   # Sensor ECU     -> Controller, Diagnostic  (battery voltage)    periodic 100 ms
ID_FUEL     = 0x103   # Sensor ECU     -> Controller, Diagnostic  (fuel level)         periodic 500 ms
ID_CTRL_CMD = 0x200   # Controller ECU -> Actuator, Diagnostic    (fan/lamps/buzzer)   periodic 50 ms + on change
ID_ALARM    = 0x201   # Controller ECU -> Actuator, Diagnostic    (alarm set/clear)    event
ID_CTRL_ST  = 0x210   # Controller ECU -> Diagnostic              (mode, alarm mask)   periodic 100 ms
ID_ACT_ST   = 0x300   # Actuator ECU   -> Controller, Diagnostic  (actuator feedback)  periodic 100 ms
ID_DIAG_RPT = 0x7E0   # Diagnostic ECU -> all                     (DTC report)         event
ID_TEST_CMD = 0x7E1   # Diagnostic ECU -> Sensor ECU              (fault injection)    event

MESSAGES = {
    ID_MOTION:   dict(name="MOTION_DATA",    tx="Sensor",     rx="Controller, Diagnostic", kind="Periodic",  period_ms=20,  dlc=6),
    ID_COOLANT:  dict(name="COOLANT_TEMP",   tx="Sensor",     rx="Controller, Diagnostic", kind="Periodic",  period_ms=100, dlc=2),
    ID_BATTERY:  dict(name="BATTERY_VOLT",   tx="Sensor",     rx="Controller, Diagnostic", kind="Periodic",  period_ms=100, dlc=2),
    ID_FUEL:     dict(name="FUEL_LEVEL",     tx="Sensor",     rx="Controller, Diagnostic", kind="Periodic",  period_ms=500, dlc=1),
    ID_CTRL_CMD: dict(name="CTRL_COMMAND",   tx="Controller", rx="Actuator, Diagnostic",   kind="Periodic + on change", period_ms=50, dlc=4),
    ID_ALARM:    dict(name="ALARM_EVENT",    tx="Controller", rx="Actuator, Diagnostic",   kind="Event",     period_ms=None, dlc=3),
    ID_CTRL_ST:  dict(name="CTRL_STATUS",    tx="Controller", rx="Diagnostic",             kind="Periodic",  period_ms=100, dlc=4),
    ID_ACT_ST:   dict(name="ACTUATOR_STATUS",tx="Actuator",   rx="Controller, Diagnostic", kind="Periodic",  period_ms=100, dlc=5),
    ID_DIAG_RPT: dict(name="DIAG_FAULT_RPT", tx="Diagnostic", rx="All (HMI/log)",          kind="Event",     period_ms=None, dlc=4),
    ID_TEST_CMD: dict(name="TEST_COMMAND",   tx="Diagnostic", rx="Sensor",                 kind="Event",     period_ms=None, dlc=2),
}

# ---- Supervision: expected period and timeout (= 3 x period) in seconds ----
EXPECTED = {  # id: (period, timeout)
    ID_MOTION: (0.020, 0.060), ID_COOLANT: (0.100, 0.300), ID_BATTERY: (0.100, 0.300),
    ID_FUEL: (0.500, 1.500), ID_CTRL_CMD: (0.050, 0.150), ID_CTRL_ST: (0.100, 0.300),
    ID_ACT_ST: (0.100, 0.300),
}
SRC_OF = {ID_MOTION: 1, ID_COOLANT: 1, ID_BATTERY: 1, ID_FUEL: 1,
          ID_CTRL_CMD: 2, ID_ALARM: 2, ID_CTRL_ST: 2, ID_ACT_ST: 3}   # 1 Sensor, 2 Controller, 3 Actuator
SRC_NAME = {1: "Sensor ECU", 2: "Controller ECU", 3: "Actuator ECU", 4: "Diagnostic ECU"}

# ---- Plausibility limits ----
LIM_SPEED = (0.0, 300.0)      # km/h
LIM_RPM   = (0.0, 8000.0)     # rpm
LIM_TEMP  = (-40.0, 150.0)    # degC
LIM_VOLT  = (6.0, 18.0)       # V
LIM_FUEL  = (0.0, 100.0)      # %

# ---- Modes / alarms ----
MODE_NORMAL, MODE_DEGRADED, MODE_SAFE = 0, 1, 2
MODE_NAME = {0: "NORMAL", 1: "DEGRADED", 2: "SAFE"}
A_SENSOR_TIMEOUT, A_COOLANT_INVALID, A_BATT_INVALID = 0x01, 0x02, 0x03
A_OVERTEMP_WARN, A_OVERTEMP_CRIT, A_BATT_LOW, A_BATT_HIGH, A_FUEL_LOW = 0x10, 0x11, 0x12, 0x13, 0x14
A_ACT_TIMEOUT = 0x20
ALARM_NAME = {0x01: "SENSOR_TIMEOUT", 0x02: "COOLANT_INVALID", 0x03: "BATTERY_INVALID",
              0x10: "OVERTEMP_WARNING", 0x11: "OVERTEMP_CRITICAL", 0x12: "BATTERY_LOW",
              0x13: "BATTERY_HIGH", 0x14: "FUEL_LOW", 0x20: "ACTUATOR_TIMEOUT"}
LAMP_TEMP, LAMP_BATT, LAMP_FUEL, LAMP_GENERAL = 0x01, 0x02, 0x04, 0x08
FAN_NAME = {0: "OFF", 1: "LOW", 2: "HIGH"}

# ---- Test command codes (0x7E1, byte 1) ----
T_CLEAR, T_TEMP_INVALID, T_BATT_INVALID, T_MUTE, T_TEMP_HOT, T_TEMP_CRIT = 0, 1, 2, 3, 4, 5


def worst_case_bits(n):   # 11-bit ID frame incl. worst-case stuffing and 3-bit interframe space
    return 47 + 8 * n + (34 + 8 * n - 1) // 4


def nominal_bits(n):      # no stuffing
    return 47 + 8 * n


def _c(v, lo, hi):
    return max(lo, min(hi, int(round(v))))


# ---- Pack / unpack (Intel / little-endian) ----
def pack_motion(speed, rpm, ctr, status=0):
    return struct.pack("<HHBB", _c(speed / 0.01, 0, 65535), _c(rpm / 0.25, 0, 65535), ctr & 0xFF, status & 0xFF)

def unpack_motion(d):
    s, r, c, st = struct.unpack("<HHBB", d[:6]); return s * 0.01, r * 0.25, c, st

def pack_coolant(t):  return struct.pack("<h", _c(t / 0.1, -32768, 32767))
def unpack_coolant(d): return struct.unpack("<h", d[:2])[0] * 0.1
def pack_battery(v):  return struct.pack("<H", _c(v / 0.01, 0, 65535))
def unpack_battery(d): return struct.unpack("<H", d[:2])[0] * 0.01
def pack_fuel(p):     return struct.pack("<B", _c(p / 0.5, 0, 255))
def unpack_fuel(d):   return struct.unpack("<B", d[:1])[0] * 0.5   # raw > 200 => > 100 % => invalid

def pack_ctrl_cmd(fan, lamps, buzzer, ctr): return struct.pack("<BBBB", fan, lamps, buzzer, ctr & 0xFF)
def unpack_ctrl_cmd(d): return struct.unpack("<BBBB", d[:4])
def pack_alarm(code, sev, state): return struct.pack("<BBB", code, sev, state)
def unpack_alarm(d): return struct.unpack("<BBB", d[:3])
def pack_ctrl_status(mode, mask, ctr): return struct.pack("<BHB", mode, mask, ctr & 0xFF)
def unpack_ctrl_status(d): return struct.unpack("<BHB", d[:4])
def pack_act_status(fan, lamps, buzzer, ack, local_safe): return struct.pack("<BBBBB", fan, lamps, buzzer, ack & 0xFF, local_safe)
def unpack_act_status(d): return struct.unpack("<BBBBB", d[:5])
def pack_diag_report(dtc, src, status): return struct.pack("<HBB", dtc, src, status)
def unpack_diag_report(d): return struct.unpack("<HBB", d[:4])
def pack_test_cmd(target, cmd): return struct.pack("<BB", target, cmd)
def unpack_test_cmd(d): return struct.unpack("<BB", d[:2])
