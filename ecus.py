"""Hardware-independent ECU logic. Each ECU only needs: send(can_id, data), on_frame(now, id, data), tick(now)."""
import math
from can_matrix import *


class ECUBase:
    name = "ECU"

    def __init__(self, send):
        self.send = send
        self.events = []
        self.alive = True
        self.t_start = 0.0
        self._next = {}

    def start(self, now):
        self.t_start = now
        self._next = {}

    def ev(self, now, key, text):
        self.events.append({"t": round(now, 4), "ecu": self.name, "key": key, "text": text})

    def due(self, now, key, period):
        nxt = self._next.setdefault(key, self.t_start)
        if now + 1e-9 >= nxt:
            nxt += period
            self._next[key] = nxt if nxt > now else now + period
            return True
        return False

    def on_frame(self, now, cid, data): pass
    def tick(self, now): pass


# --------------------------------------------------------------------------- Sensor ECU
class SensorECU(ECUBase):
    name = "Sensor ECU"

    def start(self, now):
        super().start(now)
        self.ctr = 0
        self.fault_temp = None      # None or forced value
        self.fault_batt = None
        self.mute = False

    def signals(self, now):
        t = now - self.t_start
        speed = min(90.0, 3.0 * t)
        rpm = 800.0 + 30.0 * speed
        temp = min(92.0, 60.0 + t)
        batt = 13.80 + 0.05 * math.sin(0.5 * t)
        fuel = max(0.0, 75.0 - 0.01 * t)
        if self.fault_temp is not None: temp = self.fault_temp
        if self.fault_batt is not None: batt = self.fault_batt
        return speed, rpm, temp, batt, fuel

    def on_frame(self, now, cid, data):
        if cid != ID_TEST_CMD: return
        target, cmd = unpack_test_cmd(data)
        if target != 1: return
        if cmd == T_CLEAR:
            self.fault_temp = self.fault_batt = None
            if self.mute: self.ev(now, "MUTE_OFF", "Sensor ECU transmission restored")
            self.mute = False
            self.ev(now, "FAULT_CLEAR", "Injected faults cleared")
        elif cmd == T_TEMP_INVALID: self.fault_temp = 200.0; self.ev(now, "FAULT_TEMP_ON", "Injected coolant temperature = 200 degC")
        elif cmd == T_BATT_INVALID: self.fault_batt = 100.0; self.ev(now, "FAULT_BATT_ON", "Injected battery voltage = 100 V")
        elif cmd == T_TEMP_HOT:     self.fault_temp = 108.0; self.ev(now, "FAULT_HOT_ON", "Injected coolant temperature = 108 degC")
        elif cmd == T_TEMP_CRIT:    self.fault_temp = 118.0; self.ev(now, "FAULT_CRIT_ON", "Injected coolant temperature = 118 degC")
        elif cmd == T_MUTE:         self.mute = True; self.ev(now, "MUTE_ON", "Sensor ECU transmission suppressed")

    def tick(self, now):
        if self.mute: return
        speed, rpm, temp, batt, fuel = self.signals(now)
        if self.due(now, "motion", 0.020):
            self.send(ID_MOTION, pack_motion(speed, rpm, self.ctr)); self.ctr = (self.ctr + 1) & 0xFF
        if self.due(now, "coolant", 0.100): self.send(ID_COOLANT, pack_coolant(temp))
        if self.due(now, "battery", 0.100): self.send(ID_BATTERY, pack_battery(batt))
        if self.due(now, "fuel", 0.500):    self.send(ID_FUEL, pack_fuel(fuel))


# --------------------------------------------------------------------------- Controller ECU
class ControllerECU(ECUBase):
    name = "Controller ECU"
    RX_TIMEOUT = {ID_MOTION: 0.060, ID_COOLANT: 0.300, ID_BATTERY: 0.300, ID_FUEL: 1.500}
    ACT_TIMEOUT = 0.300
    SAFE_ALARMS = {A_SENSOR_TIMEOUT, A_COOLANT_INVALID, A_OVERTEMP_CRIT}
    DEGRADED_ALARMS = {A_BATT_INVALID, A_OVERTEMP_WARN, A_ACT_TIMEOUT}

    def start(self, now):
        super().start(now)
        self.vals, self.last_rx, self.alarms = {}, {}, set()
        self.mode, self.fan, self.lamps, self.buzzer = MODE_NORMAL, 0, 0, 0
        self.cmd_ctr = self.st_ctr = 0
        self.last_sent = None
        self.last_act = None

    def on_frame(self, now, cid, data):
        try:
            if cid == ID_MOTION:   self.vals[cid] = unpack_motion(data)[:2]
            elif cid == ID_COOLANT: self.vals[cid] = unpack_coolant(data)
            elif cid == ID_BATTERY: self.vals[cid] = unpack_battery(data)
            elif cid == ID_FUEL:    self.vals[cid] = unpack_fuel(data)
            elif cid == ID_ACT_ST:  self.last_act = now; return
            else: return
            self.last_rx[cid] = now
        except Exception:
            pass

    def _alarm(self, now, code, active, sev):
        if active and code not in self.alarms:
            self.alarms.add(code); self.send(ID_ALARM, pack_alarm(code, sev, 1))
            self.ev(now, f"ALARM_SET:{code:#04x}", f"Alarm {ALARM_NAME[code]} set")
        elif not active and code in self.alarms:
            self.alarms.discard(code); self.send(ID_ALARM, pack_alarm(code, sev, 0))
            self.ev(now, f"ALARM_CLR:{code:#04x}", f"Alarm {ALARM_NAME[code]} cleared")

    def evaluate(self, now):
        timed = {c for c, tmo in self.RX_TIMEOUT.items() if now - self.last_rx.get(c, self.t_start) > tmo}
        self._alarm(now, A_SENSOR_TIMEOUT, bool(timed), 3)
        # coolant
        tc = self.vals.get(ID_COOLANT)
        c_ok = tc is not None and ID_COOLANT not in timed and LIM_TEMP[0] <= tc <= LIM_TEMP[1]
        self._alarm(now, A_COOLANT_INVALID, tc is not None and ID_COOLANT not in timed and not c_ok, 3)
        if c_ok:
            self._alarm(now, A_OVERTEMP_WARN, tc >= 105 or (A_OVERTEMP_WARN in self.alarms and tc >= 100), 2)
            self._alarm(now, A_OVERTEMP_CRIT, tc >= 115 or (A_OVERTEMP_CRIT in self.alarms and tc >= 110), 3)
        # battery
        bv = self.vals.get(ID_BATTERY)
        b_ok = bv is not None and ID_BATTERY not in timed and LIM_VOLT[0] <= bv <= LIM_VOLT[1]
        self._alarm(now, A_BATT_INVALID, bv is not None and ID_BATTERY not in timed and not b_ok, 2)
        if b_ok:
            self._alarm(now, A_BATT_LOW, bv < 11.5 or (A_BATT_LOW in self.alarms and bv < 11.8), 2)
            self._alarm(now, A_BATT_HIGH, bv > 15.0 or (A_BATT_HIGH in self.alarms and bv > 14.7), 2)
        # fuel
        fl = self.vals.get(ID_FUEL)
        if fl is not None and ID_FUEL not in timed and fl <= LIM_FUEL[1]:
            self._alarm(now, A_FUEL_LOW, fl <= 10 or (A_FUEL_LOW in self.alarms and fl <= 12), 1)
        # actuator supervision
        self._alarm(now, A_ACT_TIMEOUT, now - (self.last_act if self.last_act is not None else self.t_start) > self.ACT_TIMEOUT, 2)
        # mode
        mode = MODE_SAFE if self.alarms & self.SAFE_ALARMS else (MODE_DEGRADED if self.alarms & self.DEGRADED_ALARMS else MODE_NORMAL)
        if mode != self.mode:
            self.ev(now, f"MODE:{MODE_NAME[mode]}", f"Controller switched {MODE_NAME[self.mode]} -> {MODE_NAME[mode]}")
            self.mode = mode
        # fan (hysteresis)
        if self.mode == MODE_SAFE: fan = 2
        elif c_ok:
            if self.fan == 2:   fan = 2 if tc >= 95 else (1 if tc >= 85 else 0)
            elif self.fan == 1: fan = 2 if tc >= 100 else (1 if tc >= 85 else 0)
            else:               fan = 2 if tc >= 100 else (1 if tc >= 90 else 0)
        else: fan = self.fan
        lamps = 0
        if self.alarms & {A_COOLANT_INVALID, A_OVERTEMP_WARN, A_OVERTEMP_CRIT, A_SENSOR_TIMEOUT}: lamps |= LAMP_TEMP
        if self.alarms & {A_BATT_INVALID, A_BATT_LOW, A_BATT_HIGH}: lamps |= LAMP_BATT
        if A_FUEL_LOW in self.alarms: lamps |= LAMP_FUEL
        if self.mode != MODE_NORMAL: lamps |= LAMP_GENERAL
        buzzer = 1 if self.mode == MODE_SAFE else 0
        if fan != self.fan:
            self.ev(now, f"FAN:{FAN_NAME[fan]}", f"Fan command {FAN_NAME[self.fan]} -> {FAN_NAME[fan]}")
        self.fan, self.lamps, self.buzzer = fan, lamps, buzzer
        if (fan, lamps, buzzer) != self.last_sent:
            self._send_cmd(now); self._next["cmd"] = now + 0.050

    def _send_cmd(self, now):
        self.cmd_ctr = (self.cmd_ctr + 1) & 0xFF
        self.send(ID_CTRL_CMD, pack_ctrl_cmd(self.fan, self.lamps, self.buzzer, self.cmd_ctr))
        self.last_sent = (self.fan, self.lamps, self.buzzer)

    def tick(self, now):
        if self.due(now, "eval", 0.010): self.evaluate(now)
        if self.due(now, "cmd", 0.050): self._send_cmd(now)
        if self.due(now, "stat", 0.100):
            mask = 0
            for a in self.alarms: mask |= 1 << list(ALARM_NAME).index(a)
            self.st_ctr = (self.st_ctr + 1) & 0xFF
            self.send(ID_CTRL_ST, pack_ctrl_status(self.mode, mask & 0xFFFF, self.st_ctr))


# --------------------------------------------------------------------------- Actuator ECU
class ActuatorECU(ECUBase):
    name = "Actuator ECU"
    CMD_TIMEOUT = 0.150

    def start(self, now):
        super().start(now)
        self.cmd = (0, 0, 0); self.ack = 0; self.last_cmd = None
        self.local_safe = False
        self.fan, self.lamps, self.buzzer = 0, 0, 0

    def on_frame(self, now, cid, data):
        if cid == ID_CTRL_CMD:
            fan, lamps, buz, ctr = unpack_ctrl_cmd(data)
            self.cmd, self.ack, self.last_cmd = (fan, lamps, buz), ctr, now

    def tick(self, now):
        ref = self.last_cmd if self.last_cmd is not None else self.t_start
        ls = now - ref > self.CMD_TIMEOUT
        if ls != self.local_safe:
            self.local_safe = ls
            self.ev(now, "LOCAL_SAFE_ON" if ls else "LOCAL_SAFE_OFF",
                    "Controller command lost: fan HIGH, lamp + buzzer ON" if ls else "Controller commands resumed")
        prev = (self.fan, self.lamps, self.buzzer)
        if ls: self.fan, self.lamps, self.buzzer = 2, self.cmd[1] | LAMP_GENERAL, 1
        else:  self.fan, self.lamps, self.buzzer = self.cmd
        if (self.fan, self.lamps, self.buzzer) != prev:
            self.ev(now, f"OUT_FAN:{FAN_NAME[self.fan]}", f"Outputs applied: fan {FAN_NAME[self.fan]}, lamps 0b{self.lamps:04b}, buzzer {'ON' if self.buzzer else 'OFF'}")
        if self.due(now, "stat", 0.100):
            self.send(ID_ACT_ST, pack_act_status(self.fan, self.lamps, self.buzzer, self.ack, int(self.local_safe)))


# --------------------------------------------------------------------------- Diagnostic ECU (+ console HMI)
class DiagnosticECU(ECUBase):
    name = "Diagnostic ECU"
    ACTION = {1: "Controller expected to enter SAFE mode (fan HIGH, lamp, buzzer)",
              2: "Actuator expected to enter local safe state",
              3: "Controller flags ACTUATOR_TIMEOUT, DEGRADED mode"}

    def start(self, now):
        super().start(now)
        self.last_rx, self.counts, self.dtcs, self.vals = {}, {}, {}, {}
        self.lost_frames = 0; self.prev_ctr = None; self.malformed = 0
        self.log = []; self.mode = None; self.act = None; self.alarms_active = {}

    def inject(self, cmd): self.send(ID_TEST_CMD, pack_test_cmd(1, cmd))

    def _raise(self, now, dtc, src, text, sev="WARNING"):
        if dtc in self.dtcs: return
        self.dtcs[dtc] = now
        self.send(ID_DIAG_RPT, pack_diag_report(dtc, src, 1))
        act = self.ACTION.get(src, "-") if (dtc & 0xF000) == 0xC000 else "Value rejected by consumers"
        self.log.append(dict(t=now, level=sev, text=text, dtc=dtc, action=act))
        self.ev(now, f"DTC_SET:{dtc:#06x}", text)

    def _clear(self, now, dtc, src, text):
        if dtc not in self.dtcs: return
        del self.dtcs[dtc]
        self.send(ID_DIAG_RPT, pack_diag_report(dtc, src, 0))
        self.log.append(dict(t=now, level="INFO", text=text, dtc=dtc, action="-"))
        self.ev(now, f"DTC_CLR:{dtc:#06x}", text)

    def _range(self, now, dtc, src, ok, label):
        if not ok: self._raise(now, dtc, src, f"{label} out of range", "ERROR")
        else:      self._clear(now, dtc, src, f"{label} back in range")

    def on_frame(self, now, cid, data):
        self.counts[cid] = self.counts.get(cid, 0) + 1
        if cid in EXPECTED:
            self.last_rx[cid] = now
            self._clear(now, 0xC000 | cid, SRC_OF[cid], f"Message 0x{cid:03X} received again")
        try:
            if cid == ID_MOTION:
                s, r, c, _ = unpack_motion(data); self.vals["speed"], self.vals["rpm"] = s, r
                if self.prev_ctr is not None: self.lost_frames += (c - self.prev_ctr - 1) & 0xFF
                self.prev_ctr = c
                self._range(now, 0xB100, 1, LIM_SPEED[0] <= s <= LIM_SPEED[1] and LIM_RPM[0] <= r <= LIM_RPM[1], "Speed/RPM")
            elif cid == ID_COOLANT:
                v = unpack_coolant(data); self.vals["coolant"] = v
                self._range(now, 0xB101, 1, LIM_TEMP[0] <= v <= LIM_TEMP[1], f"Coolant temperature ({v:.1f} degC)")
            elif cid == ID_BATTERY:
                v = unpack_battery(data); self.vals["batt"] = v
                self._range(now, 0xB102, 1, LIM_VOLT[0] <= v <= LIM_VOLT[1], f"Battery voltage ({v:.2f} V)")
            elif cid == ID_FUEL:
                v = unpack_fuel(data); self.vals["fuel"] = v
                self._range(now, 0xB103, 1, LIM_FUEL[0] <= v <= LIM_FUEL[1], "Fuel level")
            elif cid == ID_CTRL_CMD: self.vals["cmd"] = unpack_ctrl_cmd(data)[:3]
            elif cid == ID_ALARM:
                code, sev, st = unpack_alarm(data)
                self.log.append(dict(t=now, level="INFO", text=f"Controller alarm {ALARM_NAME.get(code, hex(code))} {'SET' if st else 'CLEARED'}", dtc=None, action="-"))
            elif cid == ID_CTRL_ST:
                mode = unpack_ctrl_status(data)[0]
                if mode != self.mode:
                    self.log.append(dict(t=now, level="INFO", text=f"Controller mode {MODE_NAME.get(self.mode, '-')} -> {MODE_NAME[mode]}", dtc=None, action="-"))
                    self.ev(now, f"MODE_SEEN:{MODE_NAME[mode]}", "Mode change observed on bus")
                    self.mode = mode
            elif cid == ID_ACT_ST: self.act = unpack_act_status(data)
        except Exception:
            self.malformed += 1

    def tick(self, now):
        if not self.due(now, "mon", 0.010): return
        for cid, (period, tmo) in EXPECTED.items():
            if now - self.last_rx.get(cid, self.t_start) > tmo:
                src = SRC_OF[cid]
                self._raise(now, 0xC000 | cid, src, f"{SRC_NAME[src]} timeout: message 0x{cid:03X} missing > {int(tmo*1000)} ms", "WARNING")

    def dashboard(self, now):
        v = self.vals
        g = lambda k, f: (f.format(v[k]) if k in v else "--")
        cmd = v.get("cmd", (0, 0, 0)); act = self.act or (0, 0, 0, 0, 0)
        lines = [
            "=" * 60,
            f" VEHICLE MONITORING SYSTEM         t = {now:8.2f} s",
            "=" * 60,
            f" Speed {g('speed','{:6.1f}')} km/h   RPM {g('rpm','{:7.1f}')}   Fuel {g('fuel','{:5.1f}')} %",
            f" Coolant {g('coolant','{:6.1f}')} degC   Battery {g('batt','{:6.2f}')} V",
            f" Controller mode : {MODE_NAME.get(self.mode, '--')}",
            f" Actuators       : Fan {FAN_NAME.get(act[0], '-')}  Lamps 0b{act[1]:04b}  Buzzer {'ON' if act[2] else 'OFF'}  LocalSafe {'YES' if act[4] else 'no'}",
            f" Active DTCs     : {', '.join(f'0x{d:04X}' for d in sorted(self.dtcs)) or 'none'}",
            "-" * 60, " Recent events:"]
        for e in self.log[-5:]:
            lines.append(f"  [{e['t']:8.3f}] {e['level']:<7} {e['text']}")
        lines.append("=" * 60)
        return "\n".join(lines)
