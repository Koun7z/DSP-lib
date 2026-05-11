
import argparse
import pandas as pd
from matplotlib import pyplot as plt


def get_reference_from_output(fname):
    """
    Read reference input path from the first comment line:
    # input_file=/path/to/reference.csv
    """
    with open(fname, "r") as f:
        first_line = f.readline().strip()

    if first_line.startswith("#"):
        return first_line.split(" ", 1)[1]

    return None


# ===============================
# Source configuration
# ===============================

DATA_SOURCES = {
    "RepoIMU": {
        "sep": ";",
        "skiprows": 2,
    },
    "synth": {
        "sep": ";",
        "skiprows": 2,
    },
}

# Central registry of supported filters
AVAILABLE_FILTERS = [
    "nc",
    "ekf",
    "madgwick",
    "mahony",
]


# ===============================
# Main
# ===============================


def main():
    parser = argparse.ArgumentParser(
        description="Plot AHRS output vs reference data"
    )

    parser.add_argument(
        "--source",
        choices=DATA_SOURCES.keys(),
        required=True,
        help="Data source (e.g. synth, RepoIMU)",
    )

    parser.add_argument(
        "--filter",
        nargs="+",
        required=False,
        default="all",
        help='Filter name(s) or "all" (e.g. ekf madgwick | all)',
    )

    args = parser.parse_args()

    # Resolve filter list
    if "all" in args.filter:
        filters = AVAILABLE_FILTERS
    else:
        filters = args.filter

    source_cfg = DATA_SOURCES[args.source]

    LW_MAIN = 1.6
    LW_REF = 1.2

    PALETTE = {
        "main": ["#1f77b4", "#d62728", "#2ca02c", "#9467bd"],
        "ref": ["#7ab8e8", "#f4908f", "#8dd58d", "#c9aee3"],
    }

    ref_columns = [
        "Time_s",
        "Vicon_W", "Vicon_X", "Vicon_Y", "Vicon_Z",
        "Acc_X", "Acc_Y", "Acc_Z",
        "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Mag_X", "Mag_Y", "Mag_Z",
    ]

    ahrs_columns = ["W", "X", "Y", "Z"]

    for filt in filters:
        fname = f"data/ahrs_{filt}_{args.source}_output.csv"
        title = f"{filt.upper()} filter ({args.source})"

        # -------------------------------
        # Read reference input
        # -------------------------------
        ref_path = get_reference_from_output(fname)
        if ref_path is None:
            print(f"Skipping {filt}: no reference file in {fname}")
            continue

        print(f"\nFilter     : {filt}")
        print(f"AHRS file  : {fname}")
        print(f"Reference  : {ref_path}")

        df_ref = pd.read_csv(
            ref_path,
            sep=source_cfg["sep"],
            skiprows=source_cfg["skiprows"],
            names=ref_columns,
        )

        df_ahrs = pd.read_csv(
            fname,
            sep=",",
            comment="#",
            names=ahrs_columns,
        )

        # -------------------------------
        # Plot
        # -------------------------------
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))

        ax.set_prop_cycle(color=PALETTE["main"])
        ax.plot(
            df_ref["Time_s"],
            df_ahrs[["W", "X", "Y", "Z"]],
            linewidth=LW_MAIN,
        )

        ax.set_prop_cycle(color=PALETTE["ref"])
        ax.plot(
            df_ref["Time_s"],
            df_ref[["Vicon_W", "Vicon_X", "Vicon_Y", "Vicon_Z"]],
            "--",
            linewidth=LW_REF,
        )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Quaternion Components")
        ax.set_title(title)
        ax.legend(["W", "X", "Y", "Z"])
        ax.grid(True)

    plt.show()

if __name__ == "__main__":
    main()
