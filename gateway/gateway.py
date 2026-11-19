"""Gateway service: reads the CAN bus, decodes the frames, stores everything in
SQLite and records diagnostic events (node online/offline, error flags,
lost frames, temperature sensor problems).

  python3 gateway.py                       # vcan0, database ../data/gateway.db
  python3 gateway.py --channel can0        # real bus
"""
import argparse
import os
import time

import can

import protocol as p
from storage import Storage

try:
    from detector import Detector
except ImportError:          # scikit-learn not installed: run without AI
    Detector = None

HEARTBEAT_TIMEOUT_S = 3.0
FLAG_NAMES = {p.ERR_IMU: "IMU error", p.ERR_TEMP: "temperature sensor error",
              p.ERR_QUEUE_FULL: "queue full", p.ERR_CAN_TX: "CAN TX error"}


class NodeState:
    def __init__(self):
        self.online = False
        self.last_hb = 0.0
        self.flags = 0
        self.last_counter = None
        self.temp_status = p.TEMP_OK


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--db", default=os.path.join(here, "..", "data", "gateway.db"))
    ap.add_argument("--no-ai", action="store_true", help="record data only (to build a training set)")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.db)), exist_ok=True)
    store = Storage(args.db)
    bus = can.Bus(interface=args.interface, channel=args.channel)
    nodes = {1: NodeState(), 2: NodeState()}
    stats = {"frames": 0, "unknown": 0}
    store.event(time.time(), None, "info", "gateway_start", f"listening on {args.channel}")

    detector = None
    if not args.no_ai and Detector is not None:
        try:
            detector = Detector()
            print(f"AI anomaly detection ON (threshold {detector.threshold:.3f})")
        except FileNotFoundError:
            print("No trained model yet (ml/models/): AI detection OFF")
    last_ai = time.time()
    print(f"Gateway listening on {args.channel}, database {os.path.abspath(args.db)}")

    last_report = time.time()
    try:
        while True:
            msg = bus.recv(timeout=0.5)
            now = time.time()
            if msg is not None and not msg.is_error_frame:
                d = p.decode(msg.arbitration_id, bytes(msg.data))
                if d is None:
                    stats["unknown"] += 1
                else:
                    stats["frames"] += 1
                    store.add(now, d)
                    handle_diagnostics(now, d, nodes[d["node"]], store)

            # node offline detection
            for nid, st in nodes.items():
                if st.online and now - st.last_hb > HEARTBEAT_TIMEOUT_S:
                    st.online = False
                    store.event(now, nid, "error", "node_offline",
                                f"no heartbeat for {HEARTBEAT_TIMEOUT_S:.0f} s")

            if detector is not None and now - last_ai >= 1.0:
                detector.step(store.db, now, store)
                last_ai = now

            if now - last_report > 5:
                print(f"frames={stats['frames']} unknown={stats['unknown']} "
                      f"online={[n for n, s in nodes.items() if s.online]}")
                last_report = now
    except KeyboardInterrupt:
        pass
    finally:
        store.event(time.time(), None, "info", "gateway_stop", "")
        store.close()
        bus.shutdown()


def handle_diagnostics(now, d, st, store):
    node = d["node"]
    if d["type"] == "heartbeat":
        if not st.online:
            st.online = True
            store.event(now, node, "info", "node_online", f"uptime {d['uptime_s']} s")
        st.last_hb = now
        new_flags = d["flags"] & ~st.flags
        cleared = st.flags & ~d["flags"]
        for bit, name in FLAG_NAMES.items():
            if new_flags & bit:
                store.event(now, node, "error", "error_flag_set", name)
            if cleared & bit:
                store.event(now, node, "info", "error_flag_cleared", name)
        st.flags = d["flags"]
    elif d["type"] == "vibration":
        if st.last_counter is not None:
            lost = (d["counter"] - st.last_counter - 1) & 0xFF
            if lost:
                store.event(now, node, "warning", "frames_lost",
                            f"{lost} vibration frame(s) missing")
        st.last_counter = d["counter"]
    elif d["type"] == "temperature" and d["status"] != st.temp_status:
        if d["status"] != p.TEMP_OK:
            store.event(now, node, "warning", "temp_sensor",
                        {1: "sensor not found", 2: "read error"}.get(d["status"], "unknown"))
        st.temp_status = d["status"]


if __name__ == "__main__":
    main()
