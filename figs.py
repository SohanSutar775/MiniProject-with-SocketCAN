import json
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
plt.rcParams["font.family"] = "Liberation Serif"

def box(ax, x, y, w, h, text, fc="#E8EEF7", ec="#1F3A5F", fs=11, bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.02,rounding_size=0.06", fc=fc, ec=ec, lw=1.4))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs, fontweight="bold" if bold else "normal")

def arrow(ax, p, q, text=None, off=(0, 0.08), fs=9):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle="-|>", mutation_scale=14, lw=1.3, color="#333"))
    if text: ax.text((p[0] + q[0]) / 2 + off[0], (p[1] + q[1]) / 2 + off[1], text, ha="center", fontsize=fs, style="italic")

# ---------------- Fig 1 functional block diagram
fig, ax = plt.subplots(figsize=(9, 4.6)); ax.set_xlim(0, 12); ax.set_ylim(0, 6.2); ax.axis("off")
ax.text(1.4, 5.9, "INPUTS (sensing)", ha="center", fontsize=11, fontweight="bold")
ax.text(6.0, 5.9, "PROCESSING", ha="center", fontsize=11, fontweight="bold")
ax.text(10.6, 5.9, "OUTPUTS (actuation)", ha="center", fontsize=11, fontweight="bold")
ins = ["Vehicle speed", "Engine speed (RPM)", "Coolant temperature", "Battery voltage", "Fuel level"]
for i, t in enumerate(ins):
    y = 4.7 - i * 0.95; box(ax, 0.1, y, 2.6, 0.7, t, fc="#FFF4D6", ec="#8A6D00", fs=10); arrow(ax, (2.7, y + 0.35), (4.1, 3.05 if i > 2 else 3.1 + (2 - i) * 0.0 + 0.0))
box(ax, 4.1, 2.0, 3.8, 2.3, "Monitoring & decision logic\n\nplausibility checks\nthreshold + hysteresis\nmode manager\n(NORMAL / DEGRADED / SAFE)", fc="#E8EEF7", fs=10)
box(ax, 4.5, 0.2, 3.0, 1.1, "Diagnostics\nDTC · event log · HMI", fc="#E6F4EA", ec="#1E6B34", fs=10)
arrow(ax, (6.0, 2.0), (6.0, 1.3), "supervise", off=(0.7, 0))
outs = ["Cooling fan\n(OFF / LOW / HIGH)", "Warning lamps\n(temp / batt / fuel / general)", "Buzzer"]
for i, t in enumerate(outs):
    y = 3.9 - i * 1.15; box(ax, 9.2, y, 2.7, 0.8, t, fc="#FCE4E4", ec="#8B1E1E", fs=9.5); arrow(ax, (7.9, 3.15), (9.2, y + 0.4))
box(ax, 9.2, 0.2, 2.7, 0.9, "Driver / technician\n(console HMI)", fc="#EDE7F6", ec="#4A2C7A", fs=10)
arrow(ax, (7.5, 0.75), (9.2, 0.65), "status, DTCs", off=(0, 0.1))
arrow(ax, (9.2, 0.45), (7.5, 0.45), None); ax.text(8.35, 0.05, "fault-injection keys", ha="center", fontsize=9, style="italic")
fig.savefig("fig/fig1_functional.png", dpi=200, bbox_inches="tight"); plt.close(fig)

# ---------------- Fig 2 ECU architecture
fig, ax = plt.subplots(figsize=(9, 4.9)); ax.set_xlim(0, 12); ax.set_ylim(0, 6.6); ax.axis("off")
ax.plot([0.4, 11.6], [3.3, 3.3], color="#111", lw=5, solid_capstyle="round")
ax.text(6, 1.05, "Shared bus: SocketCAN interface vcan0 (CAN 2.0A, 500 kbit/s)", ha="center", fontsize=11, fontweight="bold")
nodes = [(0.3, "Sensor ECU\n(sensor_ecu.py)", "speed, RPM, coolant,\nbattery, fuel\n+ fault injection", "#FFF4D6", "#8A6D00"),
         (3.2, "Controller ECU\n(controller_ecu.py)", "plausibility, thresholds,\nmode manager,\nfan/lamp/buzzer logic", "#E8EEF7", "#1F3A5F"),
         (6.1, "Actuator ECU\n(actuator_ecu.py)", "fan, lamps, buzzer,\nfeedback, local\nsafe state", "#FCE4E4", "#8B1E1E"),
         (9.0, "Diagnostic ECU + HMI\n(diagnostic_ecu.py)", "bus monitor, DTCs,\nevent log, dashboard,\ntest commands", "#E6F4EA", "#1E6B34")]
for x, t, d, fc, ec in nodes:
    top = x < 6  # alternate not needed; all above for clarity
    box(ax, x, 4.6, 2.7, 1.7, t + "\n\n" + d, fc=fc, ec=ec, fs=9.5)
    ax.plot([x + 1.35, x + 1.35], [4.6, 3.3], color="#111", lw=2)
labels = [(0.3, "TX 0x100 0x101\n0x102 0x103\nRX 0x7E1"), (3.2, "TX 0x200 0x201 0x210\nRX 0x100-0x103 0x300"), (6.1, "TX 0x300\nRX 0x200 0x201"), (9.0, "TX 0x7E0 0x7E1\nRX all frames")]
for x, t in labels:
    ax.text(x + 1.35, 1.9, t, ha="center", va="center", fontsize=9, bbox=dict(boxstyle="round", fc="white", ec="#888"))
    ax.plot([x + 1.35, x + 1.35], [3.3, 2.5], color="#111", lw=1, ls=":")
ax.text(6, 0.45, "Each ECU = separate OS process; no shared memory — all interaction is through CAN frames.", ha="center", fontsize=10, style="italic")
fig.savefig("fig/fig2_ecu.png", dpi=200, bbox_inches="tight"); plt.close(fig)

# ---------------- Fig 3 timeline
R = json.load(open("results.json")); S = R["samples"]
t = [s["t"] for s in S]
fig, ax = plt.subplots(4, 1, figsize=(9, 7.2), sharex=True, gridspec_kw=dict(height_ratios=[1.2, 1, 1, 1]))
ax[0].plot(t, [s["coolant"] if s["coolant"] is not None else float("nan") for s in S], color="#B03A2E", lw=1.5)
ax[0].set_ylabel("Coolant\nseen by Diag (°C)"); ax[0].set_ylim(0, 215)
ax[1].step(t, [s["mode"] if s["ctrl_alive"] else float("nan") for s in S], where="post", color="#1F3A5F"); ax[1].set_yticks([0, 1, 2]); ax[1].set_yticklabels(["NORMAL", "DEGR.", "SAFE"]); ax[1].set_ylabel("Controller\nmode")
ax[2].step(t, [s["fan"] if s["act_alive"] else float("nan") for s in S], where="post", color="#1E6B34", label="actuator fan"); ax[2].set_yticks([0, 1, 2]); ax[2].set_yticklabels(["OFF", "LOW", "HIGH"]); ax[2].set_ylabel("Fan state")
ax[3].step(t, [len(s["dtcs"]) for s in S], where="post", color="#7D3C98"); ax[3].set_ylabel("Active\nDTCs"); ax[3].set_xlabel("Time (s)")
spans = [(10, 14, "C2 sensor\nsilent"), (18, 22, "C3a 200 °C"), (24, 28, "C3b 100 V"), (32, 38, "C4/C5 actuator\ndown"), (42, 46, "controller\ndown"), (50, 53, "108 °C"), (53, 56, "118 °C")]
for a in ax:
    for i, (x0, x1, _) in enumerate(spans): a.axvspan(x0, x1, color="#F5B041" if i % 2 == 0 else "#85C1E9", alpha=0.25, lw=0)
    a.grid(alpha=0.25)
for x0, x1, lab in spans: ax[0].text((x0 + x1) / 2, 6, lab, ha="center", va="bottom", fontsize=7.5)
fig.suptitle("System response over the 60 s fault-injection scenario (virtual-time simulation)", fontsize=11)
fig.tight_layout(); fig.savefig("fig/fig3_timeline.png", dpi=200); plt.close(fig)
print("figures ok")
