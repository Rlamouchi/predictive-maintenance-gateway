"""Simulates the 2 STM32 nodes on a CAN bus (virtual vcan0 or real can0).

Normal motor: low vibration. With --fault, one motor starts to vibrate more
and to heat up after a delay, so the gateway, dashboard and AI can be tested
before the real hardware is ready.

Examples:
  python3 simulator.py                            # 2 normal motors on vcan0
  python3 simulator.py --fault 2 --fault-after 60 # motor 2 degrades after 60 s
"""
import argparse
import math
import random
import time

import can

import protocol as p


class MotorNode:
    def __init__(self, node, base_rms, base_temp):
        self.node = node
        self.base_rms = base_rms
        self.base_temp = base_temp
        self.temp = base_temp
        self.counter = 0
        self.start = time.time()
        self.fault_level = 0.0  # 0 = healthy, 1 = strong fault

    def load(self):
        """Normal operation is not constant: the motor load changes slowly (+-15 %)."""
        t = time.time() - self.start
        return 1 + 0.15 * math.sin(2 * math.pi * t / (90 + 20 * self.node))

    def vibration(self):
        t = time.time() - self.start
        rms = self.base_rms * self.load() * (1 + 0.05 * math.sin(t)) + random.gauss(0, 8)
        rms += self.fault_level * 450 + self.fault_level * random.gauss(0, 40)
        rms = max(rms, 1)
        peak = rms * random.uniform(2.6, 3.2) * (1 + 0.6 * self.fault_level)
        self.counter = (self.counter + 1) & 0xFF
        return p.encode_vibration(int(rms), int(peak), self.counter)

    def temperature(self):
        target = self.base_temp + 3 * (self.load() - 1) / 0.15 + 18 * self.fault_level
        self.temp += (target - self.temp) * 0.05 + random.gauss(0, 0.05)
        return p.encode_temperature(self.temp, p.TEMP_OK)

    def heartbeat(self):
        return p.encode_heartbeat(1, 0, int(time.time() - self.start))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interface", default="socketcan")
    ap.add_argument("--channel", default="vcan0")
    ap.add_argument("--fault", type=int, choices=[1, 2], help="node that degrades")
    ap.add_argument("--fault-after", type=float, default=60.0, help="seconds")
    ap.add_argument("--fault-ramp", type=float, default=60.0,
                    help="seconds to go from healthy to full fault")
    args = ap.parse_args()

    bus = can.Bus(interface=args.interface, channel=args.channel)
    nodes = {1: MotorNode(1, 150, 38.0), 2: MotorNode(2, 170, 40.0)}
    t0 = time.time()
    tick = 0
    print(f"Simulating nodes 1 and 2 on {args.channel}. Ctrl+C to stop.")
    try:
        while True:
            elapsed = time.time() - t0
            if args.fault and elapsed > args.fault_after:
                lvl = min((elapsed - args.fault_after) / args.fault_ramp, 1.0)
                nodes[args.fault].fault_level = lvl
            for n in nodes.values():
                bus.send(can.Message(arbitration_id=p.make_id(n.node, p.MSG_VIBRATION),
                                     data=n.vibration(), is_extended_id=False))
                if tick % 10 == 0:
                    bus.send(can.Message(arbitration_id=p.make_id(n.node, p.MSG_TEMPERATURE),
                                         data=n.temperature(), is_extended_id=False))
                    bus.send(can.Message(arbitration_id=p.make_id(n.node, p.MSG_HEARTBEAT),
                                         data=n.heartbeat(), is_extended_id=False))
            tick += 1
            time.sleep(0.1)  # vibration message every 100 ms, as in PROTOCOL.md
    except KeyboardInterrupt:
        pass
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
