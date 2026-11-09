"""Real SocketCAN backend (Linux, python stdlib only). Runs one ECU as an independent process."""
import select, socket, struct, sys, time

FMT = "=IB3x8s"   # struct can_frame: can_id(4) dlc(1) pad(3) data(8)


class SocketCanBus:
    def __init__(self, ifname="vcan0"):
        self.s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.s.bind((ifname,))

    def send(self, cid, data):
        data = bytes(data)
        self.s.send(struct.pack(FMT, cid, len(data), data.ljust(8, b"\x00")))

    def recv(self, timeout):
        if not select.select([self.s], [], [], timeout)[0]: return None
        cid, dlc, data = struct.unpack(FMT, self.s.recv(16))
        return cid & socket.CAN_SFF_MASK, data[:dlc]


def run_ecu(ecu_cls, ifname=None, hmi=False):
    ifname = ifname or (sys.argv[1] if len(sys.argv) > 1 else "vcan0")
    bus = SocketCanBus(ifname)
    ecu = ecu_cls(bus.send)
    ecu.start(time.monotonic())
    print(f"[{ecu.name}] started on {ifname}", flush=True)
    last_draw = 0.0
    try:
        while True:
            fr = bus.recv(0.001)
            while fr:
                ecu.on_frame(time.monotonic(), *fr)
                fr = bus.recv(0)
            now = time.monotonic()
            ecu.tick(now)
            if hmi:
                if now - last_draw > 0.5:
                    last_draw = now
                    print("\033[2J\033[H" + ecu.dashboard(now - ecu.t_start))
                    print(" keys: 1=bad temp(200C) 2=bad batt(100V) 3=mute sensor 4=108C 5=118C 0=clear  (Enter)")
                if select.select([sys.stdin], [], [], 0)[0]:
                    k = sys.stdin.readline().strip()
                    if k in "012345" and k: ecu.inject(int(k))
    except KeyboardInterrupt:
        print(f"\n[{ecu.name}] stopped")
