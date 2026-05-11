#!/usr/bin/env python3

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ===============================
# Quaternion utilities
# ===============================

from Quaternion import quat_mul, quat_conj, quat_normalize, rotate_vector, quat_from_omega

# ===============================
# Main generation
# ===============================

def generate_data(duration, dt):
    time = np.arange(0.0, duration, dt)

    # Angular velocity amplitudes (rad/s)
    Ax = np.deg2rad(10)   # roll right
    Ay = np.deg2rad(6)    # pitch up
    Az = np.deg2rad(15)   # yaw right

    # Frequencies (Hz)
    fx, fy, fz = 0.07, 0.05, 0.09

    # NED environment vectors
    g_ned = np.array([0.0, 0.0, 9.81])  # gravity (Down positive)
    m_ned = np.array([-0.33, 0.20, -0.88])
    m_ned /= np.linalg.norm(m_ned)

    q = np.array([1.0, 0.0, 0.0, 0.0])  # body → NED
    rows = []

    for t in time:
        omega_body = np.array([
            Ax * np.sin(2*np.pi*fx*t),
            Ay * np.sin(2*np.pi*fy*t),
            Az * np.sin(2*np.pi*fz*t),
        ])

        q = quat_normalize(q)

        acc = rotate_vector(quat_conj(q), g_ned)
        gyro = omega_body
        mag = rotate_vector(quat_conj(q), m_ned)

        rows.append([
            t,
            q[0], q[1], q[2], q[3],
            acc[0], acc[1], acc[2],
            gyro[0], gyro[1], gyro[2],
            mag[0], mag[1], mag[2],
        ])

        dq = quat_from_omega(omega_body, dt)
        q = quat_mul(q, dq)

    columns = [
        "Time_s",
        "Q_W", "Q_X", "Q_Y", "Q_Z",
        "Acc_X", "Acc_Y", "Acc_Z",
        "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Mag_X", "Mag_Y", "Mag_Z",
    ]

    return pd.DataFrame(rows, columns=columns)

# ===============================
# Plotting
# ===============================

def plot_data(df):
    t = df["Time_s"]

    fig, axs = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    # Orientation
    axs[0].plot(t, df[["Q_W", "Q_X", "Q_Y", "Q_Z"]])
    axs[0].set_ylabel("Quaternion")
    axs[0].legend(["W", "X", "Y", "Z"])
    axs[0].set_title("Reference Orientation")

    # Accelerometer
    axs[1].plot(t, df[["Acc_X", "Acc_Y", "Acc_Z"]])
    axs[1].set_ylabel("Accel (m/s²)")
    axs[1].legend(["X", "Y", "Z"])
    axs[1].set_title("IMU Acceleration (Body, NED)")

    # Gyroscope
    axs[2].plot(t, df[["Gyro_X", "Gyro_Y", "Gyro_Z"]])
    axs[2].set_ylabel("Gyro (rad/s)")
    axs[2].legend(["X", "Y", "Z"])
    axs[2].set_title("IMU Gyroscope (Right-hand, Body Frame)")

    # Magnetometer
    axs[3].plot(t, df[["Mag_X", "Mag_Y", "Mag_Z"]])
    axs[3].set_ylabel("Mag (-)")
    axs[3].legend(["X", "Y", "Z"])
    axs[3].set_title("IMU Magnetometer (Body Frame)")
    axs[3].set_xlabel("Time (s)")

    plt.tight_layout()
    plt.show()

# ===============================
# Entry point
# ===============================

def main():
    parser = argparse.ArgumentParser(description="Generate noise-free synthetic NED IMU data")
    parser.add_argument("--duration", type=float, default=180.0,
                        help="Duration in seconds (default: 180)")
    parser.add_argument("--dt", type=float, default=0.01,
                        help="Sampling period (default: 0.01 s)")
    parser.add_argument("--output", type=str, default="./data/ahrs_synth_data.csv",
                        help="Output CSV filename")
    parser.add_argument("--plot", action="store_true",
                        help="Plot reference and sensor data")

    args = parser.parse_args()

    df = generate_data(args.duration, args.dt)

    with open(args.output, "w") as f:
        f.write("Time (s);Ref Orientation;;;;IMU Acceleration;;;IMU Gyroscope;;;IMU Magnetometer;;;\n")
        f.write(";W;X;Y;Z;X;Y;Z;X;Y;Z;X;Y;Z;\n")
        df.to_csv(
            f,
            sep=";",
            index=False,
            header=False,
            float_format="%.7g",
        )

    print(f"Saved synthetic data to: {args.output}")

    if args.plot:
        plot_data(df)

if __name__ == "__main__":
    main()