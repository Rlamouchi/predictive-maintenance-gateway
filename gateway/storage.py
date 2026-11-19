"""SQLite storage: measurements, node states and a diagnostic event history."""
import sqlite3
import time

SCHEMA = """
CREATE TABLE IF NOT EXISTS vibration (
    ts REAL NOT NULL, node INTEGER NOT NULL,
    rms_mg INTEGER, peak_mg INTEGER, counter INTEGER);
CREATE INDEX IF NOT EXISTS idx_vib ON vibration(node, ts);

CREATE TABLE IF NOT EXISTS temperature (
    ts REAL NOT NULL, node INTEGER NOT NULL, temp_c REAL, status INTEGER);
CREATE INDEX IF NOT EXISTS idx_temp ON temperature(node, ts);

CREATE TABLE IF NOT EXISTS heartbeat (
    ts REAL NOT NULL, node INTEGER NOT NULL,
    state INTEGER, flags INTEGER, uptime_s INTEGER);
CREATE INDEX IF NOT EXISTS idx_hb ON heartbeat(node, ts);

-- Anomaly score computed by the AI model, once per second per node
CREATE TABLE IF NOT EXISTS anomaly (
    ts REAL NOT NULL, node INTEGER NOT NULL, score REAL, abnormal INTEGER);
CREATE INDEX IF NOT EXISTS idx_an ON anomaly(node, ts);

-- Diagnostic history: faults, events and state changes
CREATE TABLE IF NOT EXISTS events (
    ts REAL NOT NULL, node INTEGER, severity TEXT, type TEXT, detail TEXT);
CREATE INDEX IF NOT EXISTS idx_ev ON events(ts);
"""


class Storage:
    def __init__(self, path):
        self.db = sqlite3.connect(path, check_same_thread=False)
        self.db.execute("PRAGMA journal_mode=WAL")  # dashboard can read while we write
        self.db.executescript(SCHEMA)
        self.pending = 0
        self.last_commit = time.time()

    def add(self, ts, d):
        if d["type"] == "vibration":
            self.db.execute("INSERT INTO vibration VALUES (?,?,?,?,?)",
                            (ts, d["node"], d["rms_mg"], d["peak_mg"], d["counter"]))
        elif d["type"] == "temperature":
            self.db.execute("INSERT INTO temperature VALUES (?,?,?,?)",
                            (ts, d["node"], d["temp_c"], d["status"]))
        elif d["type"] == "heartbeat":
            self.db.execute("INSERT INTO heartbeat VALUES (?,?,?,?,?)",
                            (ts, d["node"], d["state"], d["flags"], d["uptime_s"]))
        self.pending += 1
        self.maybe_commit()

    def anomaly(self, ts, node, score, abnormal):
        self.db.execute("INSERT INTO anomaly VALUES (?,?,?,?)", (ts, node, score, int(abnormal)))
        self.pending += 1
        self.maybe_commit()

    def event(self, ts, node, severity, etype, detail):
        self.db.execute("INSERT INTO events VALUES (?,?,?,?,?)",
                        (ts, node, severity, etype, detail))
        self.pending += 1
        self.maybe_commit(force=True)

    def maybe_commit(self, force=False):
        now = time.time()
        if force or now - self.last_commit > 0.5:
            self.db.commit()
            self.pending = 0
            self.last_commit = now

    def close(self):
        self.db.commit()
        self.db.close()
