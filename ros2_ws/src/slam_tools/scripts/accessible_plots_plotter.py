# accessible_plots.py
from accessible_plots_calculation import (
    quaternion_to_yaw,
    compute_yaw_error,
    compute_regression,
    compute_yaw_derivative,
    compute_kde
)

def plot_yaw_error(dlio, lio_interp, t_dlio, save_path):
    yaw_error = compute_yaw_error(dlio, lio_interp)

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

def plot_error_vs_yaw(t, err_xy, yaw_err, out):
    slope, intercept, r_value, _, _ = compute_regression(yaw_err, err_xy)

    plt.figure(figsize=(10, 6))
    plt.scatter(yaw_err, err_xy, s=5, alpha=0.3, label="Samples")

    x_line = np.linspace(min(yaw_err), max(yaw_err), 200)
    plt.plot(
        x_line,
        slope*x_line + intercept,
        color="red",
        linewidth=2,
        label=f"Fit (R={r_value:.2f})"
    )

    plt.xlabel("Yaw Error [rad]")
    plt.ylabel("XY Error [m]")
    plt.title("Error vs. Yaw Drift")
    plt.grid(True, linestyle="--", alpha=0.3)
    plt.legend()
    plt.savefig(out + "/error_vs_yaw.png", dpi=200)
    plt.close()

def plot_error_violin(err_norm, save_path):
    y, density = compute_kde(err_norm)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.fill_betweenx(
        y, -density, density,
        facecolor="#4477AA", edgecolor="black",
        linewidth=1.2, alpha=0.9
    )

    q1, q2, q3 = np.percentile(err_norm, [25, 50, 75])
    ax.plot([0], [q2], "o", color="white", markersize=10)
    ax.hlines([q1, q2, q3], xmin=-0.4, xmax=0.4,
              colors="white", linestyles="--", linewidth=2)

    ax.set_title("Distribution of XY Error (Violin Plot)")
    ax.set_ylabel("XY Error [m]")
    ax.set_xlim(-0.5, 0.5)
    ax.set_xticks([0])
    ax.set_xticklabels(["DLIO"])
    ax.grid(True, linestyle="--", alpha=0.3)

    plt.tight_layout()
    plt.savefig(save_path, dpi=200)
    plt.close()
