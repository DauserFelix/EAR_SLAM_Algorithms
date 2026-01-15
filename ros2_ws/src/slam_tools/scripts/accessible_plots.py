import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress


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

def plot_accessible_hexbin(x, y, xlabel, ylabel, title, save_path):
    plt.figure(figsize=(8,6))
    hb = plt.hexbin(
        x, y,
        gridsize=60,
        cmap="viridis",
        mincnt=1
    )
    plt.colorbar(hb, label="Count")
    plt.xlabel(xlabel, fontsize=14)
    plt.ylabel(ylabel, fontsize=14)
    plt.title(title, fontsize=16)
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()

def plot_error_vs_yaw(t, err_xy, yaw_err, out):
    """Scatter + Regression für Fehler vs. Yaw Drift."""
    slope, intercept, r_value, p_value, std_err = linregress(yaw_err, err_xy)

    plt.figure(figsize=(10, 6))
    plt.scatter(yaw_err, err_xy, s=5, alpha=0.3, label="Samples")

    x_line = np.linspace(min(yaw_err), max(yaw_err), 200)
    plt.plot(x_line, slope * x_line + intercept, color="red",
             linewidth=2, label=f"Linear Fit (R={r_value:.2f})")

    plt.xlabel("Yaw Error [rad]")
    plt.ylabel("XY Error [m]")
    plt.title("Error vs. Yaw Drift")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()

    plt.savefig(out + "/error_vs_yaw.png", dpi=200)
    plt.close()
    print("✓ error_vs_yaw.png gespeichert.")


def plot_yaw_heatmap(err_xy, yaw_err, out):
    """2D-Hexbin-Heatmap Error vs Yaw Error."""
    plt.figure(figsize=(10, 6))
    hb = plt.hexbin(yaw_err, err_xy, gridsize=60, cmap="viridis", mincnt=1)

    plt.xlabel("Yaw Error [rad]")
    plt.ylabel("XY Error [m]")
    plt.title("2D Heatmap: XY Error vs. Yaw Drift")
    cb = plt.colorbar(hb)
    cb.set_label("Count")

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.savefig(out + "/error_vs_yaw_heatmap.png", dpi=200)
    plt.close()
    print("✓ error_vs_yaw_heatmap.png gespeichert.")



def plot_yaw_derivative_vs_error(t, yaw_err, err_xy, out):
    """Yaw-Fehleränderung vs Error (zeigt plötzliche Yaw-Sprünge)."""

    dt = np.gradient(t)
    dyaw = np.gradient(yaw_err) / dt  # yaw rate error

    plt.figure(figsize=(10, 6))
    hb = plt.hexbin(np.abs(dyaw), err_xy,
                    gridsize=70, cmap="plasma", mincnt=1)

    plt.xlabel("|d(Yaw Error)/dt|  [rad/s]")
    plt.ylabel("XY Error [m]")
    plt.title("Error vs. Yaw Error Change Rate")
    cb = plt.colorbar(hb)
    cb.set_label("Count")

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.savefig(out + "/error_vs_yaw_derivative.png", dpi=200)
    plt.close()
    print("✓ error_vs_yaw_derivative.png gespeichert.")


def plot_error_violin(err_norm, save_path):
    import numpy as np
    import matplotlib.pyplot as plt
    from scipy.stats import gaussian_kde

    # --- KDE für Violin ermitteln ---
    data = np.array(err_norm)
    kde = gaussian_kde(data)
    y = np.linspace(data.min(), data.max(), 300)
    density = kde(y)

    # Normieren, damit die Violine seitlich passt
    density = density / density.max() * 0.4  # Breite der Violine

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(8, 6))

    # Barrierefreie Farben
    face_color = "#4477AA"
    edge_color = "black"

    # Violine links und rechts
    ax.fill_betweenx(
        y, -density, density,
        facecolor=face_color,
        edgecolor=edge_color,
        linewidth=1.2,
        alpha=0.9,
        label="DLIO Error Distribution"
    )

    # Quartile
    q1, q2, q3 = np.percentile(data, [25, 50, 75])
    ax.plot([0], [q2], 'o', color="white", markersize=10)     # Median-Punkt
    ax.hlines([q1, q2, q3], xmin=-0.4, xmax=0.4,
              colors="white", linestyles="--", linewidth=2)

    # Achsenlabels
    ax.set_title("Distribution of XY Error (Violin Plot)")
    ax.set_xlabel("DLIO")
    ax.set_ylabel("XY Error [m]")

    ax.set_xlim(-0.5, 0.5)
    ax.set_xticks([0])
    ax.set_xticklabels(["DLIO"])

    # Y-Grid
    ax.grid(True, linestyle="--", alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close()

