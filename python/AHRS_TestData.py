import numpy as np
import pandas as pd
from matplotlib import pyplot as plt

LW_MAIN = 1.6
LW_REF  = 1.2

PALETTE = {
    'main': ['#1f77b4', '#d62728', '#2ca02c', '#9467bd'],
    'ref':  ['#7ab8e8', '#f4908f', '#8dd58d', '#c9aee3']

}

columns = [
    "Time_s",
    "Vicon_W", "Vicon_X", "Vicon_Y", "Vicon_Z",
    "Acc_X", "Acc_Y", "Acc_Z",
    "Gyro_X", "Gyro_Y", "Gyro_Z",
    "Mag_X", "Mag_Y", "Mag_Z",
]

df_ref = pd.read_csv(
    "../Test/RepoIMU/TStick/TStick_Test02_Trial1.csv",
    sep=";",
    skiprows=2,
    names=columns
)

print(df_ref.head())
print(df_ref.shape)

columns = [
    "W", "X", "Y", "Z",
]

df_ahrs = pd.read_csv(
    "./data/ahrs_nc_output.csv",
    sep=",",
    names=columns
)

print(df_ahrs.head())
print(df_ahrs.shape)

for fname, title in [
    ("data/ahrs_nc_output.csv",  "Comparison of output of NC AQUA Filter"),
    ("data/ahrs_ekf_output.csv", "Comparison of output of EKF Filter"),
    ("data/ahrs_madgwick_output.csv", "Comparison of output of Madgwick Filter"),
]:
    df_ahrs = pd.read_csv(
        fname,
        sep=",",
        names=columns
    )

    fig, ax = plt.subplots(1, 1, figsize=(10, 8))

    ax.set_prop_cycle(color=PALETTE["main"])
    ax.plot(df_ref["Time_s"], df_ahrs[["W", "X", "Y", "Z"]], linewidth=LW_MAIN)

    ax.set_prop_cycle(color=PALETTE["ref"])
    ax.plot(df_ref["Time_s"], df_ref[["Vicon_W", "Vicon_X", "Vicon_Y", "Vicon_Z"]], "--", linewidth=LW_REF)

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Quaternion Components")
    ax.set_title(title)
    ax.legend(["W", "X", "Y", "Z"])

plt.show()