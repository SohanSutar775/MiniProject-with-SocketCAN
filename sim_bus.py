"""Virtual-time simulation of a 500 kbit/s CAN bus (priority arbitration, worst-case frame length)
driving the SAME ECU classes that run on SocketCAN. Used to compute the observations in the test report."""
import json
from can_matrix import *
from ecus import SensorECU, ControllerECU, ActuatorECU, DiagnosticECU


class SimBus:
    def __init__(self):
        self.pending, self.free_at, self.busy, self.seq = [], 0.0, 0.0, 0
        self.nodes, self.now, self.frames = {}, 0.0, []

    def attach(self, ecu):
        self.nodes[ecu.name] = ecu
        ecu.send = lambda cid, data, n=ecu.name: self._send(n, cid, data)

    def _send(self, src, cid, data):
        self.seq += 1
        self.pending.append(dict(id=cid, data=bytes(data), src=src, ready=self.now, seq=self.seq))

    def drop_from(self, name):
        self.pending = [f for f in self.pending if f["src"] != name]

    def advance(self, now_end):
        while self.pending:
            start = max(self.free_at, min(f["ready"] for f in self.pending))
            if start >= now_end: break
            f = min((f for f in self.pending if f["ready"] <= start), key=lambda f: (f["id"], f["seq"]))
            self.pending.remove(f)
            dur = worst_case_bits(len(f["data"])) / BITRATE
            end = start + dur
            self.free_at, self.busy = end, self.busy + dur
            self.frames.append(dict(id=f["id"], src=f["src"], ready=f["ready"], start=start, end=end, bits=worst_case_bits(len(f["data"]))))
            self.now = end
            for n, e in self.nodes.items():
                if n != f["src"] and e.alive: e.on_frame(end, f["id"], f["data"])


class Simulation:
    def __init__(self):
        self.bus = SimBus()
        self.sensor, self.ctrl, self.act, self.diag = SensorECU(None), ControllerECU(None), ActuatorECU(None), DiagnosticECU(None)
        self.ecus = [self.sensor, self.ctrl, self.act, self.diag]
        for e in self.ecus: self.bus.attach(e); e.start(0.0)
        self.actions, self.samples, self.marks = [], [], []

    def at(self, t, fn, label=None): self.actions.append((t, fn, label))

    def stop(self, ecu):
        ecu.alive = False; self.bus.drop_from(ecu.name)
        ecu.events.append({"t": round(self.t, 4), "ecu": ecu.name, "key": "STOPPED", "text": "ECU process stopped"})

    def restart(self, ecu):
        ecu.alive = True; ecu.start(self.t)
        ecu.events.append({"t": round(self.t, 4), "ecu": ecu.name, "key": "RESTARTED", "text": "ECU process restarted"})

    def run(self, duration, dt=0.001):
        self.actions.sort(key=lambda a: a[0]); ai = 0
        for k in range(int(round(duration / dt))):
            self.t = t = round(k * dt, 6)
            while ai < len(self.actions) and self.actions[ai][0] <= t + 1e-9:
                self.bus.now = t; self.actions[ai][1](); ai += 1
            for e in self.ecus:
                if e.alive:
                    self.bus.now = t; e.tick(t)
            self.bus.advance(t + dt)
            if k % 100 == 0:
                c, a, d = self.ctrl, self.act, self.diag
                self.samples.append(dict(t=t, mode=c.mode, fan_cmd=c.fan, fan=a.fan, lamps=a.lamps, buzzer=a.buzzer,
                                         local_safe=int(a.local_safe), dtcs=sorted(d.dtcs), coolant=d.vals.get("coolant"),
                                         batt=d.vals.get("batt"), ctrl_alive=int(c.alive), act_alive=int(a.alive)))
        return self

    def events(self):
        out = []
        for e in self.ecus: out += e.events
        return sorted(out, key=lambda x: x["t"])


def build_scenario():
    s = Simulation(); d = s.diag
    s.at(10.0, lambda: d.inject(T_MUTE));        s.at(14.0, lambda: d.inject(T_CLEAR))     # C2 missing messages
    s.at(18.0, lambda: d.inject(T_TEMP_INVALID)); s.at(22.0, lambda: d.inject(T_CLEAR))    # C3a invalid temperature
    s.at(24.0, lambda: d.inject(T_BATT_INVALID)); s.at(28.0, lambda: d.inject(T_CLEAR))    # C3b invalid battery voltage
    s.at(32.0, lambda: s.stop(s.act));           s.at(38.0, lambda: s.restart(s.act))      # C4/C5 actuator failure/recovery
    s.at(42.0, lambda: s.stop(s.ctrl));          s.at(46.0, lambda: s.restart(s.ctrl))     # extra: controller failure/recovery
    s.at(50.0, lambda: d.inject(T_TEMP_HOT));    s.at(53.0, lambda: d.inject(T_TEMP_CRIT)); s.at(56.0, lambda: d.inject(T_CLEAR))  # thresholds
    return s


if __name__ == "__main__":
    s = build_scenario().run(60.0)
    print(s.diag.dashboard(60.0)); print(len(s.bus.frames), "frames")
