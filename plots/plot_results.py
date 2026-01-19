#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import os

csv_path = '../data/simulated_baro.csv'

if not os.path.exists(csv_path):
    print(f"Data file not found: {csv_path}")
    exit(1)

df = pd.read_csv(csv_path)

fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

# Height plot
ax1 = axes[0]
ax1.plot(df['time'], df['baro_meas'], 'c.', markersize=1, alpha=0.4, label='Baro (raw)')
ax1.plot(df['time'], df['true_h'], 'k-', linewidth=1.5, label='True height')
ax1.plot(df['time'], df['ekf_h'], 'r-', linewidth=1.2, label='EKF estimate')
ax1.set_ylabel('Height [m]')
ax1.legend(loc='upper right')
ax1.grid(True, alpha=0.3)
ax1.set_title('1D EKF Height Estimation')

# Velocity plot
ax2 = axes[1]
ax2.plot(df['time'], df['true_v'], 'k-', linewidth=1.5, label='True velocity')
ax2.plot(df['time'], df['ekf_v'], 'r-', linewidth=1.2, label='EKF estimate')
ax2.set_ylabel('Velocity [m/s]')
ax2.set_xlabel('Time [s]')
ax2.legend(loc='upper right')
ax2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('height_plot.png', dpi=150)
plt.show()

print("Plot saved to height_plot.png")
