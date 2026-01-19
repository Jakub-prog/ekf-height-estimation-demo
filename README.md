# EKF Height Estimation Demo

This repository contains a minimal example of a 1D Extended Kalman Filter for height estimation using IMU acceleration (prediction) and barometric altitude (update).

The goal of this repo is not performance, but clarity:
- predict vs update separation
- role of Q, R, and P matrices
- behavior under noisy measurements

This is a simplified, illustrative example. My production work is under NDA.

## Quick Start

```bash
cd src
make
./ekf_demo
```

Output CSV will be written to `data/simulated_baro.csv`.

## Filter Design

**State vector:**
```
x = [h, v]'
```
- `h` — height above reference [m]
- `v` — vertical velocity [m/s]

**Prediction step** (IMU acceleration):
```
h_new = h + v*dt + 0.5*a*dt²
v_new = v + a*dt
```

**Update step** (barometer):
```
y = z_baro - h
K = P*H' / (H*P*H' + R)
x = x + K*y
```

## Tuning Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `Q_h` | Position process noise | 0.001 - 0.1 |
| `Q_v` | Velocity process noise | 0.01 - 0.5 |
| `R` | Barometer measurement noise | 1.0 - 5.0 |

Higher Q → faster uncertainty growth during prediction  
Higher R → measurements have lower influence during update

## Notes

- IMU acceleration is used in the prediction step and contributes to process noise Q.
- Barometer is used as an absolute measurement with noise R.
- The filter behavior can be observed by changing Q/R and inspecting the plots.
- This structure mirrors height estimators commonly used in aerial robotics, where IMU runs at high rate and barometer provides slower absolute correction.

## Simulation

The demo runs a 30-second trajectory with:
- Climb phase (0-5s)
- Coast (5-10s)
- Descent (10-15s)
- Coast (15-20s)
- Sinusoidal oscillation (20-30s)

Sensor noise is injected to simulate realistic conditions.

## File Structure

```
├── src/
│   ├── ekf.h        # Filter interface
│   ├── ekf.c        # Filter implementation
│   ├── main.c       # Demo application
│   └── Makefile
├── data/
│   └── simulated_baro.csv
└── plots/
    └── height_plot.png
```

## License

MIT
