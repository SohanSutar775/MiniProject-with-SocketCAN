"""Run the scenario and extract the observations used in the test report -> results.json"""
import json, statistics
from can_matrix import *
from sim_bus import build_scenario

def first(evs, key, t0, ecu=None):
    for e in evs:
        if e["t"] >= t0 - 1e-9 and e["key"] == key and (ecu is None or e["ecu"] == ecu): return e["t"]
    return None

def last_frame(frames, cid, before):
    c = [f["end"] for f in frames if f["id"] == cid and f["end"] <= before]
    return max(c) if c else None

def sample_at(samples, t):
    return min(samples, key=lambda s: abs(s["t"] - t))

def main():
    s = build_scenario().run(60.0)
    ev, fr, sm = s.events(), s.bus.frames, s.samples
    R = {}
    # ---------- C1 normal operation: window 2..10 s (8 s)
    W0, W1 = 2.0, 10.0
    rows = []
    for cid, m in MESSAGES.items():
        f = [x for x in fr if x["id"] == cid and W0 <= x["end"] < W1]
        lat = [(x["end"] - x["ready"]) * 1e6 for x in f]
        exp = (W1 - W0) / (m["period_ms"] / 1000) if m["period_ms"] else None
        rows.append(dict(id=f"0x{cid:03X}", name=m["name"], expected=None if exp is None else round(exp), observed=len(f),
                         rate=round(len(f) / (W1 - W0), 2), mean_lat_us=round(statistics.mean(lat), 1) if lat else None,
                         max_lat_us=round(max(lat), 1) if lat else None))
    busy = sum(x["bits"] for x in fr if W0 <= x["end"] < W1) / BITRATE
    R["c1"] = dict(rows=rows, load_sim=round(100 * busy / (W1 - W0), 3), lost_frames=s.diag.lost_frames,
                   ctrl_fan_at_9=sample_at(sm, 9)["fan"], mode_at_9=sample_at(sm, 9)["mode"], dtc_at_9=sample_at(sm, 9)["dtcs"])
    # ---------- analytic bus load
    wc = nm = 0.0; arows = []
    for cid, m in MESSAGES.items():
        if m["period_ms"]:
            r = 1000 / m["period_ms"]; w = worst_case_bits(m["dlc"]); n = nominal_bits(m["dlc"])
            wc += r * w; nm += r * n
            arows.append(dict(id=f"0x{cid:03X}", dlc=m["dlc"], rate=r, nominal_bits=n, worst_bits=w, wc_bps=r * w))
    R["analytic"] = dict(rows=arows, load_worst=round(100 * wc / BITRATE, 3), load_nominal=round(100 * nm / BITRATE, 3),
                         wc_bps=wc, nm_bps=nm)
    # ---------- C2 missing messages (sensor muted at 10 s, restored at 14 s)
    t_mute = first(ev, "MUTE_ON", 10); t_unm = first(ev, "MUTE_OFF", 14)
    c2 = dict(t_mute=t_mute, t_restore=t_unm, det=[])
    for cid in (ID_MOTION, ID_COOLANT, ID_BATTERY, ID_FUEL):
        lf = last_frame(fr, cid, t_mute); tset = first(ev, f"DTC_SET:{0xC000|cid:#06x}", 10)
        tclr = first(ev, f"DTC_CLR:{0xC000|cid:#06x}", 14)
        c2["det"].append(dict(id=f"0x{cid:03X}", last_rx=lf, timeout=EXPECTED[cid][1], dtc_set=tset,
                              latency_ms=round((tset - lf) * 1000, 1) if tset else None,
                              bound_ms=EXPECTED[cid][1] * 1000 + 10 + 0, dtc_clr=tclr))
    c2["alarm_t"] = first(ev, "ALARM_SET:0x01", 10); c2["safe_t"] = first(ev, "MODE:SAFE", 10)
    c2["fan_hi_t"] = first(ev, "FAN:HIGH", 10); c2["normal_t"] = first(ev, "MODE:NORMAL", 14)
    c2["alarm_clr_t"] = first(ev, "ALARM_CLR:0x01", 14)
    c2["state_mid"] = sample_at(sm, 12); c2["state_after"] = sample_at(sm, 15.5)
    c2["lost_after_restore"] = s.diag.lost_frames
    R["c2"] = c2
    # ---------- C3 sensor faults
    t1 = first(ev, "FAULT_TEMP_ON", 18)
    c3a = dict(t_inj=t1, dtc=first(ev, "DTC_SET:0xb101", 18), alarm=first(ev, "ALARM_SET:0x02", 18), safe=first(ev, "MODE:SAFE", 18),
               fan_hi=first(ev, "FAN:HIGH", 18), t_clear=first(ev, "FAULT_CLEAR", 22), dtc_clr=first(ev, "DTC_CLR:0xb101", 22),
               normal=first(ev, "MODE:NORMAL", 22), state=sample_at(sm, 20))
    t2 = first(ev, "FAULT_BATT_ON", 24)
    c3b = dict(t_inj=t2, dtc=first(ev, "DTC_SET:0xb102", 24), alarm=first(ev, "ALARM_SET:0x03", 24), deg=first(ev, "MODE:DEGRADED", 24),
               t_clear=first(ev, "FAULT_CLEAR", 28), dtc_clr=first(ev, "DTC_CLR:0xb102", 28), normal=first(ev, "MODE:NORMAL", 28), state=sample_at(sm, 26))
    R["c3a"], R["c3b"] = c3a, c3b
    # ---------- C4 actuator failure / C5 recovery
    t_st = first(ev, "STOPPED", 32, "Actuator ECU"); lf = last_frame(fr, ID_ACT_ST, t_st)
    ws = lambda a, b, cid: len([x for x in fr if x["id"] == cid and a <= x["end"] < b])
    c4 = dict(t_stop=t_st, last_rx=lf, dtc=first(ev, "DTC_SET:0xc300", 32), alarm=first(ev, "ALARM_SET:0x20", 32), deg=first(ev, "MODE:DEGRADED", 32),
              sensor_frames=ws(33, 38, ID_MOTION), sensor_expected=int(5 / 0.020), coolant_frames=ws(33, 38, ID_COOLANT),
              ctrl_frames=ws(33, 38, ID_CTRL_CMD), state=sample_at(sm, 35))
    t_rs = first(ev, "RESTARTED", 38, "Actuator ECU")
    c5 = dict(t_restart=t_rs, first_status=min(x["end"] for x in fr if x["id"] == ID_ACT_ST and x["end"] >= t_rs),
              dtc_clr=first(ev, "DTC_CLR:0xc300", 38), alarm_clr=first(ev, "ALARM_CLR:0x20", 38), normal=first(ev, "MODE:NORMAL", 38),
              state=sample_at(sm, 40))
    R["c4"], R["c5"] = c4, c5
    # ---------- extra: controller failure and recovery
    t_cs = first(ev, "STOPPED", 42, "Controller ECU"); lfc = last_frame(fr, ID_CTRL_CMD, t_cs)
    cx = dict(t_stop=t_cs, last_cmd=lfc, local_safe=first(ev, "LOCAL_SAFE_ON", 42), dtc_cmd=first(ev, "DTC_SET:0xc200", 42),
              dtc_st=first(ev, "DTC_SET:0xc210", 42), state=sample_at(sm, 44),
              t_restart=first(ev, "RESTARTED", 46, "Controller ECU"), local_safe_off=first(ev, "LOCAL_SAFE_OFF", 46),
              dtc_clr=first(ev, "DTC_CLR:0xc200", 46), mode_after=sample_at(sm, 48)["mode"])
    R["cx"] = cx
    # ---------- threshold tests
    th = dict(hot_inj=first(ev, "FAULT_HOT_ON", 50), hot_alarm=first(ev, "ALARM_SET:0x10", 50), hot_deg=first(ev, "MODE:DEGRADED", 50),
              hot_state=sample_at(sm, 52), crit_inj=first(ev, "FAULT_CRIT_ON", 53), crit_alarm=first(ev, "ALARM_SET:0x11", 53),
              crit_safe=first(ev, "MODE:SAFE", 53), crit_state=sample_at(sm, 55), clr=first(ev, "FAULT_CLEAR", 56),
              back_normal=first(ev, "MODE:NORMAL", 56), fan_low_first=first(ev, "FAN:LOW", 0))
    R["th"] = th
    # ---------- arbitration / latency
    allf = [(x["end"] - x["ready"]) * 1e6 for x in fr]
    R["bus"] = dict(frames=len(fr), total_load=round(100 * s.bus.busy / 60.0, 3), max_lat_us=round(max(allf), 1),
                    mean_lat_us=round(statistics.mean(allf), 1),
                    worst_by_id={f"0x{c:03X}": round(max((x["end"] - x["ready"]) * 1e6 for x in fr if x["id"] == c), 1) for c in MESSAGES})
    R["events"] = [e for e in ev if e["key"].split(":")[0] in ("MUTE_ON", "MUTE_OFF", "FAULT_TEMP_ON", "FAULT_BATT_ON", "FAULT_CLEAR", "STOPPED", "RESTARTED",
                   "DTC_SET", "DTC_CLR", "MODE", "LOCAL_SAFE_ON", "LOCAL_SAFE_OFF", "FAN")]
    # actuator output application times (end-to-end latency)
    R["e2e"] = dict(
        c2_fault=t_mute, c2_fan=first(ev, "OUT_FAN:HIGH", 10, "Actuator ECU"),
        c3a_fault=t1, c3a_fan=first(ev, "OUT_FAN:HIGH", 18, "Actuator ECU"),
        hot_fault=th["hot_inj"], hot_fan=first(ev, "OUT_FAN:HIGH", 50, "Actuator ECU"),
        c5_out=first(ev, "OUT_FAN:LOW", 38, "Actuator ECU"))
    R["samples"] = sm
    R["dashboard"] = {}
    for tt in (9.0, 12.0, 20.0, 35.0, 44.0):
        ss = build_scenario().run(tt + 0.0005); R["dashboard"][str(tt)] = ss.diag.dashboard(tt)
    json.dump(R, open("results.json", "w"), indent=1, default=str)
    return R, s

if __name__ == "__main__":
    R, s = main()
    for k in ("c1", "analytic", "c2", "c3a", "c3b", "c4", "c5", "cx", "th", "bus"):
        d = dict(R[k]); d.pop("rows", None)
        print(k, json.dumps(d, default=str)[:900])
    print(R["c1"]["rows"])
