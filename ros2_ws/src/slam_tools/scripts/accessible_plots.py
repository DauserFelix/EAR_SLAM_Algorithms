# accessible_plots.py
# Barrierefreie Plot-Utilities für wissenschaftliche Analysen

import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------
# Farbpalette (Okabe–Ito, farbsehschwächenfreundlich)
# ---------------------------------------
ACCESSIBLE_COLORS = [
    "#0072B2",  # blue
    "#D55E00",  # orange
    "#009E73",  # green
    "#CC79A7",  # pink
    "#F0E442",  # yellow
    "#56B4E9",  # sky blue
    "#E69F00",  # golden
    "#000000",  # black
]

ACCESSIBLE_LINESTYLES = ["-", "--", "-.", ":"]
ACCESSIBLE_MARKERS    = ["o", "s", "D", "^", "v", "X", "P", "*"]


# ===================================================================
# GENERISCHER BARRIEREFREIER PLOT
# ===================================================================
def plot_accessible(
    curves,
    xlabel="",
    ylabel="",
    title="",
    save_path="plot.png",
    equal_axis=False,
    downsample=1
):
    """
    curves = [
        {"x": array, "y": array, "label": "..."},
        {"x": array, "y": array, "label": "..."}
    ]
    """

    plt.figure(figsize=(10, 7))

    for i, c in enumerate(curves):
        x = c["x"][::downsample]
        y = c["y"][::downsample]

        color  = ACCESSIBLE_COLORS[i % len(ACCESSIBLE_COLORS)]
        ls     = ACCESSIBLE_LINESTYLES[i % len(ACCESSIBLE_LINESTYLES)]
        marker = ACCESSIBLE_MARKERS[i % len(ACCESSIBLE_MARKERS)]

        plt.plot(
            x, y,
            label=c.get("label", None),
            linewidth=2.8,
            color=color,
            linestyle=ls,
            marker=marker,
            markersize=5,
            markeredgecolor="black",
            markeredgewidth=0.6
        )

    plt.xlabel(xlabel, fontsize=18)
    plt.ylabel(ylabel, fontsize=18)
    plt.title(title, fontsize=20, fontweight="bold")

    plt.grid(True, linestyle="--", alpha=0.25)

    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    if equal_axis:
        plt.axis("equal")

    plt.legend(
        fontsize=16,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

    plt.tight_layout()

    # PNG + SVG
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))

    plt.close()


# ===================================================================
# BARRIEREFREIES HISTOGRAMM
# ===================================================================
def plot_histogram(err_norm, save_path="histogram.png"):
    plt.figure(figsize=(10, 7))
    plt.hist(
        err_norm,
        bins=40,
        color=ACCESSIBLE_COLORS[0],
        alpha=0.85,
        edgecolor="black",
        linewidth=1.0
    )

    plt.xlabel("XY Error [m]", fontsize=18)
    plt.ylabel("Count", fontsize=18)
    plt.title("Histogram of XY Error", fontsize=20, fontweight="bold")

    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()


# ===================================================================
# BARRIEREFREIER CDF-PLOT
# ===================================================================
def plot_cdf(sorted_err, cdf, save_path="cdf.png"):
    plt.figure(figsize=(10, 7))
    plt.plot(
        sorted_err,
        cdf,
        linewidth=2.8,
        color=ACCESSIBLE_COLORS[1],
        linestyle="-",
        marker="o",
        markersize=5,
        markeredgecolor="black",
        markeredgewidth=0.6
    )

    plt.xlabel("XY Error [m]", fontsize=18)
    plt.ylabel("CDF", fontsize=18)
    plt.title("CDF of XY Error", fontsize=20, fontweight="bold")

    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()

def plot_xy_error_heatmap(px_lio, py_lio, err_norm, save_path):
    plt.figure(figsize=(8, 7))
    hb = plt.hexbin(
        px_lio, py_lio,
        C=err_norm,
        gridsize=60,
        cmap="viridis",
        mincnt=1
    )
    plt.colorbar(hb, label="XY Error [m]")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("XY Error Heatmap")
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()

def quaternion_to_yaw(qx, qy, qz, qw):
    return np.arctan2(
        2*(qw*qz + qx*qy),
        1 - 2*(qy*qy + qz*qz)
    )

def plot_yaw_error(dlio, lio_interp, t_dlio, save_path):
    yaw_lio = quaternion_to_yaw(
        lio_interp["qx"],
        lio_interp["qy"],
        lio_interp["qz"],
        lio_interp["qw"]
    )
    yaw_dlio = quaternion_to_yaw(
        dlio["qx"].values,
        dlio["qy"].values,
        dlio["qz"].values,
        dlio["qw"].values
    )

    yaw_error = (yaw_dlio - yaw_lio)
    yaw_error = np.unwrap(yaw_error)

    plt.figure(figsize=(10, 6))
    plt.plot(t_dlio, yaw_error, color="#0072B2", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Error [rad]")
    plt.title("Yaw Drift Over Time")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()

def plot_error_vs_imu(dlio, err_norm, t_dlio, save_path):
    fig, axs = plt.subplots(3, 1, figsize=(10, 10), sharex=True)

    # Gyro z
    axs[0].plot(t_dlio, dlio["wz"], color="#009E73")
    axs[0].set_ylabel("wz [rad/s]")
    axs[0].set_title("IMU Gyro z")

    # Acceleration magnitude
    acc_mag = np.sqrt(dlio["vx"]**2 + dlio["vy"]**2 + dlio["vz"]**2)
    axs[1].plot(t_dlio, acc_mag, color="#D55E00")
    axs[1].set_ylabel("|v| [m/s]")
    axs[1].set_title("Velocity Magnitude")

    # Error norm
    axs[2].plot(t_dlio, err_norm, color="#0072B2")
    axs[2].set_ylabel("XY Error [m]")
    axs[2].set_title("XY Error Over Time")
    axs[2].set_xlabel("Time [s]")

    for ax in axs:
        ax.grid(True, linestyle="--", alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()
