import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import os

# ── Colour palette ──────────────────────────────────────────────────────────
# Each quaternion component gets a hue family.
# "solid" = filter output,  "dashed" = reference  (lighter shade)
PALETTE = {
    'w': {'main': '#1f77b4', 'ref': '#7ab8e8'},   # blue family
    'x': {'main': '#d62728', 'ref': '#f4908f'},   # red family
    'y': {'main': '#2ca02c', 'ref': '#8dd58d'},   # green family
    'z': {'main': '#9467bd', 'ref': '#c9aee3'},   # purple family
}
LW_MAIN = 1.6
LW_REF  = 1.2

def plot_quaternion(ax, t, data, ref=None, title='', labels=('w','x','y','z')):
    """Plot quaternion components with consistent palette.
    data  – [N×4] filter output  (solid line)
    ref   – [N×4] reference       (dashed, lighter)
    """
    for k, lbl in enumerate(labels):
        ax.plot(t, data[:, k],
                color=PALETTE[lbl]['main'], lw=LW_MAIN,
                linestyle='-',  label=lbl)
    if ref is not None:
        for k, lbl in enumerate(labels):
            ax.plot(t, ref[:, k],
                    color=PALETTE[lbl]['ref'], lw=LW_REF,
                    linestyle='--', label=f'{lbl} ref')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Quaternion Components')
    ax.set_title(title)
    ax.legend(ncol=2, fontsize=8)
    ax.grid(True)

# ── Time & signal ────────────────────────────────────────────────────────────
t = np.arange(0, 100.01, 0.01)

signal = np.column_stack([
    np.sin(t / 2),
    np.sin(t / 4),
    np.sin(t / 8)
])
u = signal

# ── Reference quaternion (active / point convention, ZYX) ───────────────────
R_ref = Rotation.from_euler('ZYX', signal)
q_ref = R_ref.as_quat()                                   # [x, y, z, w]
ref   = np.column_stack([q_ref[:, 3], q_ref[:, 0],
                          q_ref[:, 1], q_ref[:, 2]])      # [w, x, y, z]

np.savetxt("data/ahrs_ref_quaternion.csv", ref, delimiter=",")

# fig, ax = plt.subplots(figsize=(10, 4))
# plot_quaternion(ax, t, ref, title='Reference position as Quaternion')
# fig.tight_layout()

# ── Euler angles ─────────────────────────────────────────────────────────────
phi   = u[:, 0]
theta = u[:, 1]
psi   = u[:, 2]
dt    = np.mean(np.diff(t))

# ── Gyroscope (body-frame angular rates) ─────────────────────────────────────
dphi   = np.gradient(phi,   dt)
dtheta = np.gradient(theta, dt)
dpsi   = np.gradient(psi,   dt)

p =  dphi   - np.sin(theta) * dpsi
q =  np.cos(phi) * dtheta + np.sin(phi) * np.cos(theta) * dpsi
r = -np.sin(phi) * dtheta + np.cos(phi) * np.cos(theta) * dpsi
gyro = np.column_stack([p, q, r])

# ── Accelerometer ─────────────────────────────────────────────────────────────
g     = 9.81
accel = np.zeros((len(t), 3))

for i in range(len(t)):
    ph, th, ps = phi[i], theta[i], psi[i]
    R = np.array([
        [ np.cos(ps)*np.cos(th),
          np.cos(ps)*np.sin(th)*np.sin(ph) - np.sin(ps)*np.cos(ph),
          np.cos(ps)*np.sin(th)*np.cos(ph) + np.sin(ps)*np.sin(ph)],
        [ np.sin(ps)*np.cos(th),
          np.sin(ps)*np.sin(th)*np.sin(ph) + np.cos(ps)*np.cos(ph),
          np.sin(ps)*np.sin(th)*np.cos(ph) - np.cos(ps)*np.sin(ph)],
        [-np.sin(th),
          np.cos(th)*np.sin(ph),
          np.cos(th)*np.cos(ph)]
    ])
    accel[i, :] = R.T @ np.array([0, 0, g])

# ── Noise ─────────────────────────────────────────────────────────────────────
rng   = np.random.default_rng(42)
accel = accel + rng.random((len(t), 3)) * 0.2  - 0.1
gyro  = gyro  + rng.random((len(t), 3)) * 0.05 - 0.025

# ── Save CSVs ─────────────────────────────────────────────────────────────────
os.makedirs("data", exist_ok=True)
np.savetxt("data/sim_gyro_data.csv",  gyro,  delimiter=",")
np.savetxt("data/sim_accel_data.csv", accel, delimiter=",")

# ── IMU plots ─────────────────────────────────────────────────────────────────
GYRO_COLORS  = ['#d62728', '#2ca02c', '#1f77b4']
ACCEL_COLORS = ['#d62728', '#2ca02c', '#1f77b4']

for xlim, suffix in [(None, 'full'), ([40, 60], 'zoom')]:
    fig, axes = plt.subplots(2, 1, figsize=(10, 7))

    for k, (lbl, col) in enumerate(zip(['p (roll rate)', 'q (pitch rate)', 'r (yaw rate)'], GYRO_COLORS)):
        axes[0].plot(t, gyro[:, k], color=col, lw=LW_MAIN, label=lbl)
    axes[0].set_xlabel('Time (s)'); axes[0].set_ylabel('rad/s')
    axes[0].set_title('Simulated Gyroscope Output (Body Frame)')
    axes[0].legend(); axes[0].grid(True)
    if xlim: axes[0].set_xlim(xlim)

    for k, (lbl, col) in enumerate(zip(['ax', 'ay', 'az'], ACCEL_COLORS)):
        axes[1].plot(t, accel[:, k], color=col, lw=LW_MAIN, label=lbl)
    axes[1].set_xlabel('Time (s)'); axes[1].set_ylabel('m/s²')
    axes[1].set_title('Simulated Accelerometer Output (Body Frame)')
    axes[1].legend(); axes[1].grid(True)
    if xlim: axes[1].set_xlim(xlim)

    fig.tight_layout()

# ── Filter result plots ───────────────────────────────────────────────────────
for fname, title in [
    ("data/ahrs_nc_output.csv",  "Comparison of output of NC AQUA Filter"),
    ("data/ahrs_ekf_output.csv", "Comparison of output of EKF Filter"),
    ("data/ahrs_mahony_output.csv", "Comparison of output of Mahony Filter"),
]:
    try:
        result = np.loadtxt(fname, delimiter=",")
        fig, ax = plt.subplots(figsize=(10, 4))
        plot_quaternion(ax, t, result, ref=ref, title=title)
        fig.tight_layout()
    except FileNotFoundError:
        print(f"{fname} not found, skipping.")

plt.show()