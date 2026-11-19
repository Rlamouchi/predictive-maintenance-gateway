"""Live anomaly detection inside the gateway, using the model from ml/train.py."""
import json
import os

import joblib

import features as F

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR = os.path.join(HERE, "..", "ml", "models")
CLEAR_AFTER = 5   # consecutive normal seconds before the alarm is cleared


class Detector:
    def __init__(self, model_dir=MODEL_DIR):
        with open(os.path.join(model_dir, "anomaly_model.json")) as f:
            self.meta = json.load(f)
        m = joblib.load(os.path.join(model_dir, "anomaly_model.joblib"))
        self.scaler, self.model = m["scaler"], m["model"]
        self.names = self.meta["features"]
        self.threshold = self.meta["threshold"]
        self.alarm_after = self.meta["alarm_after"]
        self.state = {n: {"above": 0, "below": 0, "alarm": False} for n in (1, 2)}

    def step(self, con, now, store):
        """Score each node once; record scores and alarm events. Called every second."""
        for node, st in self.state.items():
            x = F.compute(con, node, now, self.names)
            if x is None:
                continue
            score = float(-self.model.score_samples(self.scaler.transform([x]))[0])
            abnormal = score > self.threshold
            store.anomaly(now, node, score, abnormal)
            if abnormal:
                st["above"] += 1; st["below"] = 0
                if not st["alarm"] and st["above"] >= self.alarm_after:
                    st["alarm"] = True
                    store.event(now, node, "error", "anomaly_detected",
                                f"score {score:.2f} > {self.threshold:.2f} "
                                f"(vibration {x[0]:.0f} mg)")
            else:
                st["below"] += 1; st["above"] = 0
                if st["alarm"] and st["below"] >= CLEAR_AFTER:
                    st["alarm"] = False
                    store.event(now, node, "info", "anomaly_cleared", f"score {score:.2f}")
