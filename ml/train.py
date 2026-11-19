"""Trains the anomaly-detection model on HEALTHY data only, then (optionally)
evaluates it on a recording that contains a fault.

  python3 ml/train.py --train-db data/normal.db
  python3 ml/train.py --train-db data/normal.db --test-db data/fault.db \
                      --fault-node 2 --fault-start 300

Why train on healthy data only: in a real factory we have hours of normal
operation but almost no recorded failures. The model learns what "normal"
looks like, and anything far from it is an anomaly.
"""
import argparse
import json
import os
import sqlite3
import sys

import joblib
import numpy as np
from sklearn.ensemble import IsolationForest
from sklearn.preprocessing import StandardScaler

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "gateway"))
import features as F  # noqa: E402

ALARM_AFTER = 3   # consecutive abnormal seconds before raising an alarm (avoids false alarms)


def build_matrix(db_path, names, step=1.0, nodes=(1, 2)):
    con = sqlite3.connect(db_path)
    t0, t1 = con.execute("SELECT MIN(ts), MAX(ts) FROM vibration").fetchone()
    if t0 is None:
        raise SystemExit(f"No vibration data in {db_path}")
    rows, meta = [], []
    t = t0 + F.TEMP_WINDOW_S
    while t <= t1:
        for n in nodes:
            x = F.compute(con, n, t, names)
            if x is not None:
                rows.append(x)
                meta.append((t - t0, n))
        t += step
    con.close()
    return np.array(rows), meta


def alarms(scores, threshold):
    """Alarm when the score is above the threshold for ALARM_AFTER points in a row."""
    out, run = [], 0
    for s in scores:
        run = run + 1 if s > threshold else 0
        out.append(run >= ALARM_AFTER)
    return np.array(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--train-db", required=True)
    ap.add_argument("--test-db")
    ap.add_argument("--fault-node", type=int)
    ap.add_argument("--fault-start", type=float, help="seconds after the start of the test recording")
    ap.add_argument("--out", default=os.path.join(HERE, "models"))
    args = ap.parse_args()

    con = sqlite3.connect(args.train_db)
    names = F.FEATURES_FULL if F.has_valid_temperature(con) else F.FEATURES_VIB_ONLY
    con.close()
    print(f"Features: {names}")

    X, _ = build_matrix(args.train_db, names)
    print(f"Training samples (healthy): {len(X)}")
    scaler = StandardScaler().fit(X)
    model = IsolationForest(n_estimators=200, contamination="auto", random_state=0)
    model.fit(scaler.transform(X))

    train_scores = -model.score_samples(scaler.transform(X))   # higher = more abnormal
    threshold = float(np.percentile(train_scores, 99.5)) * 1.05
    print(f"Anomaly threshold: {threshold:.3f}  "
          f"(healthy scores: median {np.median(train_scores):.3f}, max {train_scores.max():.3f})")

    os.makedirs(args.out, exist_ok=True)
    joblib.dump({"scaler": scaler, "model": model}, os.path.join(args.out, "anomaly_model.joblib"))
    with open(os.path.join(args.out, "anomaly_model.json"), "w") as f:
        json.dump({"features": names, "threshold": threshold, "alarm_after": ALARM_AFTER,
                   "vib_window_s": F.VIB_WINDOW_S, "temp_window_s": F.TEMP_WINDOW_S,
                   "train_samples": int(len(X))}, f, indent=2)
    print(f"Model saved in {args.out}")

    if args.test_db:
        Xt, meta = build_matrix(args.test_db, names)
        scores = -model.score_samples(scaler.transform(Xt))
        print("\n--- Evaluation on", args.test_db, "---")
        for n in (1, 2):
            idx = [i for i, m in enumerate(meta) if m[1] == n]
            t = np.array([meta[i][0] for i in idx]); s = scores[idx]
            a = alarms(s, threshold)
            if args.fault_node == n and args.fault_start is not None:
                before = a[t < args.fault_start]; after_t = t[(t >= args.fault_start) & a]
                print(f"Motor {n} (faulty): false alarms before the fault: {int(before.sum())} s "
                      f"of {len(before)} s")
                if len(after_t):
                    print(f"Motor {n} (faulty): fault detected {after_t[0] - args.fault_start:.0f} s "
                          f"after it started")
                else:
                    print(f"Motor {n} (faulty): fault NOT detected")
            else:
                print(f"Motor {n} (healthy): false alarms: {int(a.sum())} s of {len(a)} s")


if __name__ == "__main__":
    main()
