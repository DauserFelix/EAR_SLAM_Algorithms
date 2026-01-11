# accessible_plots.py
# Barrierefreie Plot-Utilities für wissenschaftliche Analysen

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
import numpy as np
from scipy.stats import (
    gaussian_kde,
    linregress
)

from helpers import (
    quaternion_to_yaw,
    quat_to_rot
)
    
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

# ------------------------------------------------------------
# Feste Styles pro Algorithmus (konsistent über ALLE Plots)
# ------------------------------------------------------------
STYLE_BY_LABEL = {
    "lio_sam":    {"color": "#000000", "linestyle": "-",  "marker": "o"},  # reference (black)
    "dlio":       {"color": "#0072B2", "linestyle": "--",  "marker": "o"},  # blue
    "fastlio":    {"color": "#D55E00", "linestyle": "-.", "marker": "s"},  # orange
    "lego_loam":  {"color": "#CC79A7", "linestyle": ":", "marker": "D"},  # green
    # falls du später mehr hast:
    # "kiss_icp": {"color": "...", "linestyle": ":", "marker": "^"},
}

DEFAULT_STYLE = {"color": "#000000", "linestyle": "-", "marker": "o"}  # fallback

def _style_for(label: str, i: int = 0):
    """
    Gibt festen Style für bekannten Label zurück.
    Fallback: nimmt Palette nach Index.
    """
    if label in STYLE_BY_LABEL:
        return STYLE_BY_LABEL[label]

    return {
        "color": ACCESSIBLE_COLORS[i % len(ACCESSIBLE_COLORS)],
        "linestyle": ACCESSIBLE_LINESTYLES[i % len(ACCESSIBLE_LINESTYLES)],
        "marker": ACCESSIBLE_MARKERS[i % len(ACCESSIBLE_MARKERS)],
    }



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
# BARRIEREFREIES MULTI-HISTOGRAM
# ===================================================================
def plot_histogram_multi(errors_dict, save_path="hist_multi.png"):
    plt.figure(figsize=(10, 7))

    for i, (name, err) in enumerate(errors_dict.items()):
        plt.hist(
            err,
            bins=40,
            alpha=0.35,
            label=name,
            color=ACCESSIBLE_COLORS[i % len(ACCESSIBLE_COLORS)],
            edgecolor="black",
            linestyle=ACCESSIBLE_LINESTYLES[i % len(ACCESSIBLE_LINESTYLES)],
            linewidth=1.4,
        )

    plt.xlabel("XY Error [m]", fontsize=18)
    plt.ylabel("Count", fontsize=18)
    plt.title("Histogram Comparison", fontsize=20, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.3)

    plt.legend(
        fontsize=14,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

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

# ===================================================================
# BARRIEREFREIER MULTI-CDF
# ===================================================================
def plot_cdf_multi(errors_dict, save_path="cdf_multi.png"):
    plt.figure(figsize=(10, 7))

    for i, (name, err) in enumerate(errors_dict.items()):
        e = np.sort(err)
        cdf = np.linspace(0, 1, len(e))

        st = _style_for(name, i)

        plt.plot(
            e, cdf,
            label=name,
            linewidth=2.8,
            color=st["color"],
            linestyle=st["linestyle"],
        )

    plt.xlabel("XY Error [m]", fontsize=18)
    plt.ylabel("CDF", fontsize=18)
    plt.title("CDF Comparison", fontsize=20, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.3)
    
    plt.xlim(left=0)
    plt.ylim(bottom=0)

    plt.legend(
        fontsize=14,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()


# ===================================================================
# BARRIEREFREIE HEATMAP
# ===================================================================
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

# ===================================================================
# BARRIEREFREIE YAW-ERROR
# ===================================================================
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

# ===================================================================
# BARRIEREFREIE ERROR VS. IMU
# ===================================================================
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


def plot_error_over_time_multi(t_ref, errors_dict, save_path="error_over_time.png"):
    plt.figure(figsize=(10, 7))

    for i, (name, err) in enumerate(errors_dict.items()):
        plt.plot(
            t_ref, err,
            label=name,
            linewidth=2.8,
            color=ACCESSIBLE_COLORS[i % len(ACCESSIBLE_COLORS)],
            linestyle=ACCESSIBLE_LINESTYLES[i % len(ACCESSIBLE_LINESTYLES)],
        )

    plt.xlabel("Time [s]", fontsize=18)
    plt.ylabel("XY Error [m]", fontsize=18)
    plt.title("Error Over Time", fontsize=20, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.3)

    plt.legend(
        fontsize=14,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()


# ===================================================================
# BARRIEREFREIE accessible HEXBIN PLOT
# ===================================================================

def plot_accessible(
    curves,
    xlabel="",
    ylabel="",
    title="",
    save_path="plot.png",
    equal_axis=False,
    downsample=30
):
    """
    curves = [
        {"x": array, "y": array, "label": "..."},
        {"x": array, "y": array, "label": "..."}
    ]
    """

    plt.figure(figsize=(10, 7))

    for i, c in enumerate(curves):
        label = c.get("label", None)

        x = c["x"][::downsample]
        y = c["y"][::downsample]

        st = _style_for(label, i)

        plt.plot(
            x, y,
            label=label,
            linewidth=2.6,
            color=st["color"],
            linestyle=st["linestyle"],
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
        fontsize=14,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()


# ===================================================================
# BARRIEREFREIER error VS YAW PLOT
# ===================================================================

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

# ===================================================================
# BARRIEREFREIE YAW HEATMAP
# ===================================================================
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

# ===================================================================
# BARRIEREFREIE YAW DERIVATIVE VS ERROR
# ===================================================================

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

# ===================================================================
# BARRIEREFREIE ERROR VIOLIN
# ===================================================================
def plot_error_violin(err_norm, save_path):

    data = np.array(err_norm)
    kde = gaussian_kde(data)
    y = np.linspace(data.min(), data.max(), 300)
    density = kde(y)
    density = density / density.max() * 0.4  # Breite der Violine

    fig, ax = plt.subplots(figsize=(8, 6))

    face_color = "#4477AA"
    edge_color = "black"

    # Violine
    ax.fill_betweenx(
        y, -density, density,
        facecolor=face_color,
        edgecolor=edge_color,
        linewidth=1.2,
        alpha=0.9,
    )

    # Quartile
    q1, q2, q3 = np.percentile(data, [25, 50, 75])

    # Linien (ohne Beschriftung)
    ax.hlines([q1, q2, q3], xmin=-0.4, xmax=0.4,
              colors="white", linestyles="--", linewidth=2)

    # Median-Punkt
    ax.plot([0], [q2], 'o', color="white", markersize=8)

    ax.set_title("Distribution of XY Error (Violin Plot)")
    ax.set_xlabel("Error")
    ax.set_ylabel("XY Error [m]")

    ax.set_xlim(-0.5, 0.5)
    ax.set_xticks([])

    ax.grid(True, linestyle="--", alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close()




# ===================================================================
# BARRIEREFREIE MULTI-VIOLIN
# ===================================================================
def plot_violin_multi(errors_dict, save_path="violin_multi.png"):
    plt.figure(figsize=(12, 7))

    data = [errors_dict[name] for name in errors_dict]
    names = list(errors_dict.keys())

    parts = plt.violinplot(
        data,
        showmeans=False,
        showmedians=False,
        showextrema=True
    )

    # Farben setzen
    for body in parts['bodies']:
        body.set_facecolor("#4477AA")
        body.set_edgecolor("black")
        body.set_alpha(0.8)

    # Quartile zeichnen – aber OHNE Beschriftung
    for i, name in enumerate(names, start=1):
        d = np.array(errors_dict[name])
        q1, q2, q3 = np.percentile(d, [25, 50, 75])

        plt.hlines(q1, i - 0.3, i + 0.3, colors="blue", linestyles="-", linewidth=1.5)
        plt.hlines(q2, i - 0.3, i + 0.3, colors="blue", linestyles="-", linewidth=1.5)
        plt.hlines(q3, i - 0.3, i + 0.3, colors="blue", linestyles="-", linewidth=1.5)

    plt.xticks(range(1, len(names) + 1), names)
    plt.ylabel("XY Error [m]")
    plt.title("Violin Plot Comparison")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()

    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()




# ===================================================================
# BARRIEREFREIE trajectory COLORED BY ERROR
# ===================================================================
def plot_trajectory_colored_by_error(px, py, err_xy, save_path):
    """
    Zeichnet die Trajektorie als durchgehende Linie,
    die entlang des Weges nach XY-Error farblich gefärbt ist.
    """

    import matplotlib.pyplot as plt
    import numpy as np
    from matplotlib.collections import LineCollection

    # Segmente erzeugen
    points = np.array([px, py]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)

    # Farbverlauf nach Fehler
    lc = LineCollection(
        segments,
        cmap="turbo",  # schöne Heatmap (besser als viridis)
        norm=plt.Normalize(err_xy.min(), err_xy.max())
    )
    lc.set_array(err_xy)
    lc.set_linewidth(3)

    # Plot
    plt.figure(figsize=(10, 7))
    ax = plt.gca()

    line = ax.add_collection(lc)
    plt.colorbar(line, label="XY Error [m]")

    ax.set_xlim(px.min() - 1, px.max() + 1)
    ax.set_ylim(py.min() - 1, py.max() + 1)
    ax.set_aspect("equal", "box")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trajectory Colored by XY Error")

    plt.grid(True, linestyle="--", alpha=0.3)
    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()

# ===================================================================
# BARRIEREFREIE 3D PLOT
# ===================================================================
def plot_trajectory_with_frames(
    x, y, z, qx, qy, qz, qw,
    save_path,
    x_ref=None, y_ref=None, z_ref=None
):
    """
    Final barrierefreier 3D-Trajektorienplot:
    - Downsampling & Glättung
    - Automatische Frame-Abstände
    - Orientierung klar sichtbar
    - Okabe–Ito Farben
    (Imports und quat_to_rot kommen von oben!)
    """

    # Farben: Okabe–Ito Palette
    c_traj = ACCESSIBLE_COLORS[0]   # blau
    c_ref  = ACCESSIBLE_COLORS[7]   # schwarz
    c_x    = ACCESSIBLE_COLORS[1]   # orange
    c_y    = ACCESSIBLE_COLORS[2]   # grün
    c_z    = ACCESSIBLE_COLORS[5]   # hellblau

    # -----------------------------
    # Downsampling der Trajektorie
    # -----------------------------
    N = len(x)
    ds_factor = max(1, N // 2500)     # ~2500 Samples max
    x_ds = x[::ds_factor]
    y_ds = y[::ds_factor]
    z_ds = z[::ds_factor]

    # -----------------------------
    # Trajektorie glätten
    # -----------------------------
    def smooth(a, n=20):
        return np.convolve(a, np.ones(n)/n, mode='same')

    xs = smooth(x_ds, 20)
    ys = smooth(y_ds, 20)
    zs = smooth(z_ds, 20)

    # -----------------------------
    # Größe der Szene für Skalierung
    # -----------------------------
    traj_extent = np.linalg.norm([
        np.max(x) - np.min(x),
        np.max(y) - np.min(y),
        np.max(z) - np.min(z)
    ])
    axis_length = traj_extent * 0.05    # 5% der Szene

    # -----------------------------
    # Frames stark ausdünnen
    # -----------------------------
    frame_step = max(60, N // 25)   # nur ~25 Frames

    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # -----------------------------
    # Referenz glätten & zeichnen
    # -----------------------------
    if x_ref is not None:
        ds_ref = max(1, len(x_ref) // 2500)

        x_ref_ds = x_ref[::ds_ref]
        y_ref_ds = y_ref[::ds_ref]
        z_ref_ds = z_ref[::ds_ref]

        # Weniger glätten (n=8 statt 20)
        x_ref_s = smooth(x_ref_ds, 8)
        y_ref_s = smooth(y_ref_ds, 8)
        z_ref_s = smooth(z_ref_ds, 8)

        ax.plot(
            x_ref_s, y_ref_s, z_ref_s,
            linestyle="-",     # durchgezogen
            linewidth=0.8,     # DEUTLICH dicker
            color=ACCESSIBLE_COLORS[3],  # kräftiges pink
            alpha=0.9,
            label="reference"
        )

    # -----------------------------
    # Geglättete Algorithmus-Trajektorie
    # -----------------------------
    ax.plot(
        xs, ys, zs,
        color=c_traj,
        linewidth=0.8,
        alpha=0.9,
        label="trajectory"
    )

    # -----------------------------
    # Frames zeichnen
    # -----------------------------
    for i in range(0, N, frame_step):
        R = quat_to_rot(qx[i], qy[i], qz[i], qw[i])  # kommt von oben
        p = np.array([x[i], y[i], z[i]])

        ax.plot(
            [p[0], p[0] + R[0,0]*axis_length],
            [p[1], p[1] + R[1,0]*axis_length],
            [p[2], p[2] + R[2,0]*axis_length],
            color=c_x, linewidth=2.2
        )
        ax.plot(
            [p[0], p[0] + R[0,1]*axis_length],
            [p[1], p[1] + R[1,1]*axis_length],
            [p[2], p[2] + R[2,1]*axis_length],
            color=c_y, linewidth=2.2
        )
        ax.plot(
            [p[0], p[0] + R[0,2]*axis_length],
            [p[1], p[1] + R[1,2]*axis_length],
            [p[2], p[2] + R[2,2]*axis_length],
            color=c_z, linewidth=2.2
        )

    # -----------------------------
    # Plot-Stil
    # -----------------------------
    ax.set_xlabel("x (m)", fontsize=14)
    ax.set_ylabel("y (m)", fontsize=14)
    ax.set_zlabel("z (m)", fontsize=14)
    ax.set_title("Trajectory with Orientation Frames", fontsize=18)

    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend(fontsize=12, framealpha=0.95, edgecolor="black")

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()
    
    
def plot_error_over_time_multi_varlen(errors_dict, save_path="error_over_time.png"):
    plt.figure(figsize=(10, 7))

    for i, (name, d) in enumerate(errors_dict.items()):
        st = _style_for(name, i)

        plt.plot(
            d["t"], d["err"],
            label=name,
            linewidth=2.8,
            color=st["color"],
            linestyle=st["linestyle"],
        )

    plt.xlabel("Time [s]", fontsize=18)
    plt.ylabel("XY Error [m]", fontsize=18)
    plt.title("Error Over Time", fontsize=20, fontweight="bold")
    plt.grid(True, linestyle="--", alpha=0.3)
    
    plt.xlim(left=0)
    plt.ylim(0, 5)   # <-- HIER: Y-Skala auf 0–5 m begrenzen

    plt.legend(
        fontsize=14,
        framealpha=0.95,
        facecolor="white",
        edgecolor="black"
    )

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.savefig(save_path.replace(".png", ".svg"))
    plt.close()
    