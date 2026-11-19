"""CAN protocol of the predictive maintenance gateway (see docs/PROTOCOL.md).

ID = 0x100 * node_id + message_type, standard 11-bit IDs, little-endian.
"""
import struct

MSG_VIBRATION = 0x01
MSG_TEMPERATURE = 0x02
MSG_HEARTBEAT = 0x03

TEMP_OK, TEMP_NOT_FOUND, TEMP_READ_ERROR = 0, 1, 2

ERR_IMU = 0x01
ERR_TEMP = 0x02
ERR_QUEUE_FULL = 0x04
ERR_CAN_TX = 0x08


def make_id(node: int, msg_type: int) -> int:
    return 0x100 * node + msg_type


def split_id(can_id: int):
    """Return (node, msg_type) for an ID of our protocol."""
    return can_id >> 8, can_id & 0xFF


# ---------- encoders (used by the simulator and by tests) ----------
def encode_vibration(rms_mg: int, peak_mg: int, counter: int) -> bytes:
    return struct.pack("<HHB", min(rms_mg, 0xFFFF), min(peak_mg, 0xFFFF), counter & 0xFF)


def encode_temperature(temp_c: float, status: int = TEMP_OK) -> bytes:
    return struct.pack("<hB", int(round(temp_c * 100)), status)


def encode_heartbeat(state: int, flags: int, uptime_s: int) -> bytes:
    return struct.pack("<BBI", state, flags, uptime_s & 0xFFFFFFFF)


# ---------- decoder (used by the gateway) ----------
def decode(can_id: int, data: bytes):
    """Decode one frame. Returns a dict, or None if the frame is not ours."""
    node, msg = split_id(can_id)
    if node not in (1, 2):
        return None
    try:
        if msg == MSG_VIBRATION and len(data) >= 5:
            rms, peak, counter = struct.unpack_from("<HHB", data)
            return {"node": node, "type": "vibration",
                    "rms_mg": rms, "peak_mg": peak, "counter": counter}
        if msg == MSG_TEMPERATURE and len(data) >= 3:
            raw, status = struct.unpack_from("<hB", data)
            return {"node": node, "type": "temperature",
                    "temp_c": raw / 100.0, "status": status}
        if msg == MSG_HEARTBEAT and len(data) >= 6:
            state, flags, uptime = struct.unpack_from("<BBI", data)
            return {"node": node, "type": "heartbeat",
                    "state": state, "flags": flags, "uptime_s": uptime}
    except struct.error:
        return None
    return None
