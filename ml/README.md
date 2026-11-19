# AI anomaly detection

The gateway learns what a **healthy** motor looks like and raises an alarm when
a motor moves away from that behaviour (Isolation Forest, scikit-learn).

## Features (computed every second, per motor) — `gateway/features.py`

| Feature | Window | Why |
|---|---|---|
| `rms_mean` | 10 s | overall vibration energy (imbalance, misalignment, wear) |
| `rms_std` | 10 s | how unstable the vibration is |
| `peak_max` | 10 s | shocks (bearing defects, loose parts) |
| `crest` = peak / RMS | 10 s | spiky vs. smooth vibration |
| `temp` | last value | overheating (friction, overload) |
| `temp_slope` | 60 s | °C per minute: heating up abnormally fast |

If no valid temperature exists (no DS18B20 connected), the model is trained on
the 4 vibration features only.

## Procedure

```bash
# 1. record ~10 minutes of HEALTHY operation (no AI while recording)
python3 gateway/gateway.py --db data/normal.db --no-ai
python3 gateway/simulator.py                      # other terminal; stop both after 10 min

# 2. record a run with a fault on motor 2 after 5 minutes
python3 gateway/gateway.py --db data/fault.db --no-ai
python3 gateway/simulator.py --fault 2 --fault-after 300 --fault-ramp 60

# 3. train on healthy data, evaluate on the fault run
python3 ml/train.py --train-db data/normal.db --test-db data/fault.db --fault-node 2 --fault-start 300

# 4. live: the gateway loads ml/models/ automatically
python3 gateway/gateway.py
```

## Alarm logic

- score = `-score_samples()` of the Isolation Forest (higher = more abnormal)
- threshold = 99.5th percentile of the healthy training scores + 5 % margin
- alarm (`anomaly_detected` event) after **3** abnormal seconds in a row,
  cleared (`anomaly_cleared`) after **5** normal seconds in a row

## Result on simulated data

Trained on 10 min healthy, tested on 10 min with a fault on motor 2 at t = 300 s:
0 false alarms on the healthy motor, 0 false alarms before the fault,
fault detected **5 s** after it started (vibration +~40 mg, still inside the
normal visual range). A separate 20-minute healthy run gave 0 false alarms.

## Limits / next steps

- one model for both motors: a motor that drifts to the other motor's normal level is not detected → one model per motor
- trained on simulated data: must be retrained on the real motor
- frequency features (FFT bands) would identify *which* fault (bearing, imbalance, misalignment)
