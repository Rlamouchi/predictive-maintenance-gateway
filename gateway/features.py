"""Features used by the anomaly-detection model.

The same code is used for training (ml/train.py) and live detection
(gateway/detector.py), so the model always sees features computed the same way.
Works with vibration messages every 100 ms (simulator) or every 1 s (STM32 firmware).
"""
import numpy as np

VIB_WINDOW_S = 10.0    # vibration statistics over the last 10 s
TEMP_WINDOW_S = 60.0   # temperature trend over the last 60 s
MIN_VIB_ROWS = 5
MIN_SPAN = 0.8          # the window must be at least 80 % full (no scoring on 2 s of data at start-up)

FEATURES_FULL = ["rms_mean", "rms_std", "peak_max", "crest", "temp", "temp_slope"]
FEATURES_VIB_ONLY = ["rms_mean", "rms_std", "peak_max", "crest"]


def compute(con, node, t_end, feature_names):
    """Feature vector for one node at time t_end, or None if not enough data."""
    vib = con.execute(
        "SELECT ts, rms_mg, peak_mg FROM vibration WHERE node=? AND ts>? AND ts<=?",
        (node, t_end - VIB_WINDOW_S, t_end)).fetchall()
    if len(vib) < MIN_VIB_ROWS or vib[-1][0] - vib[0][0] < MIN_SPAN * VIB_WINDOW_S - 1.0:
        return None
    rms = np.array([r[1] for r in vib], dtype=float)
    peak = np.array([r[2] for r in vib], dtype=float)
    f = {
        "rms_mean": rms.mean(),
        "rms_std": rms.std(),
        "peak_max": peak.max(),
        "crest": peak.max() / max(rms.mean(), 1.0),   # spiky shocks vs. steady vibration
    }
    if "temp" in feature_names:
        temp = con.execute(
            "SELECT ts, temp_c FROM temperature WHERE node=? AND status=0 AND ts>? AND ts<=? "
            "ORDER BY ts", (node, t_end - TEMP_WINDOW_S, t_end)).fetchall()
        if len(temp) < 2:
            return None
        ts = np.array([r[0] for r in temp]); tc = np.array([r[1] for r in temp])
        f["temp"] = tc[-1]
        f["temp_slope"] = np.polyfit(ts - ts[0], tc, 1)[0] * 60.0   # degC per minute
    return np.array([f[n] for n in feature_names], dtype=float)


def has_valid_temperature(con):
    row = con.execute("SELECT COUNT(*) FROM temperature WHERE status=0").fetchone()
    return row[0] > 0
