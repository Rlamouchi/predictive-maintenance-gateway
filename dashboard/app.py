"""Live monitoring dashboard (Flask). Reads the gateway SQLite database.

  python3 app.py            then open http://localhost:5000
"""
import argparse
import os
import sqlite3
import time

from flask import Flask, jsonify, render_template, request

HERE = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__)
DB_PATH = os.path.join(HERE, "..", "data", "gateway.db")
HEARTBEAT_TIMEOUT_S = 3.0


def load_threshold():
    try:
        import json
        with open(os.path.join(HERE, "..", "ml", "models", "anomaly_model.json")) as f:
            return json.load(f)["threshold"]
    except (OSError, KeyError, ValueError):
        return None


THRESHOLD = load_threshold()


def query(sql, params=()):
    con = sqlite3.connect(f"file:{DB_PATH}?mode=ro", uri=True)
    try:
        return con.execute(sql, params).fetchall()
    finally:
        con.close()


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/state")
def state():
    window = float(request.args.get("window", 120))
    now = time.time()
    since = now - window
    out = {"now": now, "nodes": {}}
    for node in (1, 2):
        vib = query("SELECT ts, rms_mg, peak_mg FROM vibration "
                    "WHERE node=? AND ts>? ORDER BY ts", (node, since))
        # keep at most ~300 points per chart
        step = max(1, len(vib) // 300)
        vib = vib[::step]
        temp = query("SELECT ts, temp_c, status FROM temperature "
                     "WHERE node=? ORDER BY ts DESC LIMIT 1", (node,))
        hb = query("SELECT ts, state, flags, uptime_s FROM heartbeat "
                   "WHERE node=? ORDER BY ts DESC LIMIT 1", (node,))
        online = bool(hb) and now - hb[0][0] < HEARTBEAT_TIMEOUT_S
        try:   # anomaly table exists only once the AI detector has run
            an = query("SELECT ts, score FROM anomaly WHERE node=? AND ts>? ORDER BY ts",
                       (node, since))
            last_ai = query("SELECT type FROM events WHERE node=? AND type IN "
                            "('anomaly_detected','anomaly_cleared') ORDER BY ts DESC LIMIT 1",
                            (node,))
        except sqlite3.OperationalError:
            an, last_ai = [], []
        out["nodes"][node] = {
            "vibration": [{"t": r[0], "rms": r[1], "peak": r[2]} for r in vib],
            "temp_c": temp[0][1] if temp and temp[0][2] == 0 else None,
            "online": online,
            "flags": hb[0][2] if hb else None,
            "uptime_s": hb[0][3] if hb else None,
            "anomaly": [{"t": r[0], "score": r[1]} for r in an],
            "alarm": bool(last_ai) and last_ai[0][0] == "anomaly_detected",
        }
    ev = query("SELECT ts, node, severity, type, detail FROM events "
               "ORDER BY ts DESC LIMIT 15")
    out["threshold"] = THRESHOLD
    out["events"] = [{"ts": r[0], "node": r[1], "severity": r[2],
                      "type": r[3], "detail": r[4]} for r in ev]
    return jsonify(out)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--db", default=DB_PATH)
    ap.add_argument("--port", type=int, default=5000)
    args = ap.parse_args()
    DB_PATH = args.db
    app.run(host="0.0.0.0", port=args.port, debug=False)
