#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt

DATA_DIR = "/data"
OUT_DIR = "/output"

DLIO_CSV = os.path.join(DATA_DIR, "dlio.csv")
LIO_CSV  = os.path.join(DATA_DIR, "lio_sam.csv")
SLAM_CSV = os.path.join(DATA_DIR, "slam_toolbox.csv")


def quaternion_to_yaw(qz, qw):
    return np.arctan2(2*(qw*qz), 1 - 2*(qz*qz))


def sync_to_reference(ref_t, src_t, src_vals):
    f = interp1d(src_t, src_vals, bounds_error=False, fill_value="extrapolate")
    return f(ref_t)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # ------------------------------------
    # Laden
    # ------------------------------------
    slam = pd.read_csv(SLAM_CSV)
    dlio = pd.read_csv(DLIO_CSV)
    lio  = pd.read_csv(LIO_CSV)

    # Zeitachsen normalisieren
    t_slam = slam["time_sec"].values - slam["time_sec"].values[0]
    t_dlio = dlio["time_sec"].values - dlio["time_sec"].values[0]
    t_lio  = lio["time_sec"].values  - lio["time_sec"].values[0]

    ref_t = t_slam  # slam_toolbox = Referenz

    # ------------------------------------
    # slam_toolbox Posen
    # ------------------------------------
    px_slam = slam["px"].values
    py_slam = slam["py"].values
    yaw_slam = slam["yaw"].values

    # ------------------------------------
    # DLIO auf SLAM synchronisieren
    # ------------------------------------
    px_dlio = sync_to_reference(ref_t, t_dlio, dlio["px"].values)
    py_dlio = sync_to_reference(ref_t, t_dlio, dlio["py"].values)
    yaw_dlio = quaternion_to_yaw(
        sync_to_reference(ref_t, t_dlio, dlio["qz"].values),
        sync_to_reference(ref_t, t_dlio, dlio["qw"].values)
    )

    # ------------------------------------
    # LIO-SAM auf SLAM synchronisieren
    # ------------------------------------
    px_lio = sync_to_reference(ref_t, t_lio, lio["px"].values)
    py_lio = sync_to_reference(ref_t, t_lio, lio["py"].values)
    yaw_lio = quaternion_to_yaw(
        sync_to_reference(ref_t, t_lio, lio["qz"].values),
        sync_to_reference(ref_t, t_lio, lio["qw"].values)
    )

    # ------------------------------------
    # Normalisieren (Startpunkt)
    # ------------------------------------
    def norm(a): return a - a[0]

    px_slam_n = norm(px_slam)
    py_slam_n = norm(py_slam)

    px_dlio_n = norm(px_dlio)
    py_dlio_n = norm(py_dlio)

    px_lio_n = norm(px_lio)
    py_lio_n = norm(py_lio)

    # ------------------------------------
    # Fehler (slam_toolbox als Referenz)
    # ------------------------------------
    err_dlio = np.sqrt((px_dlio_n - px_slam_n)**2 + (py_dlio_n - py_slam_n)**2)
    err_lio  = np.sqrt((px_lio_n  - px_slam_n)**2 + (py_lio_n - py_slam_n)**2)

    rmse_dlio = np.sqrt(np.mean(err_dlio**2))
    rmse_lio  = np.sqrt(np.mean(err_lio**2))

    # ------------------------------------
    # Speichern der Ergebnisse
    # ------------------------------------
    with open(os.path.join(OUT_DIR, "comparison_slam_dlio_lio.txt"), "w") as f:
        f.write("=== 2D Vergleich (Referenz: SLAM Toolbox) ===\n")
        f.write(f"RMSE DLIO   vs SLAM-Toolbox: {rmse_dlio:.3f} m\n")
        f.write(f"RMSE LIO-SAM vs SLAM-Toolbox: {rmse_lio:.3f} m\n")

    # ------------------------------------
    # XY Plot
    # ------------------------------------
    plt.figure()
    plt.plot(px_slam_n, py_slam_n, label="SLAM Toolbox (Ref)", linewidth=2)
    plt.plot(px_dlio_n, py_dlio_n, label="DLIO")
    plt.plot(px_lio_n,  py_lio_n,  label="LIO-SAM")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.axis("equal")
    plt.title("2D Trajectories Compared to SLAM Toolbox")
    plt.savefig(os.path.join(OUT_DIR, "traj_2d_comparison.png"))
    plt.close()

    # Fehlerplot
    plt.figure()
    plt.plot(ref_t, err_dlio, label="DLIO Error")
    plt.plot(ref_t, err_lio, label="LIO-SAM Error")
    plt.xlabel("time [s]")
    plt.ylabel("XY error [m]")
    plt.legend()
    plt.title("XY Error vs Time (slam_toolbox reference)")
    plt.savefig(os.path.join(OUT_DIR, "error_over_time.png"))
    plt.close()

    print("Vergleich abgeschlossen.")
    print(f"Output: {OUT_DIR}")


if __name__ == "__main__":
    main()
