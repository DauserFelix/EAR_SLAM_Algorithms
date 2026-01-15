#!/usr/bin/env python3
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

DATA_DIR = "/data"
OUT_DIR  = "/output"

DLIO_CSV = os.path.join(DATA_DIR, "dlio.csv")
LIO_CSV  = os.path.join(DATA_DIR, "lio_sam.csv")
SLAM_CSV = os.path.join(DATA_DIR, "slam_toolbox.csv")


def quaternion_to_yaw(qz, qw):
    return np.arctan2(2*(qw*qz), 1 - 2*(qz*qz))


def interpolate_to(ref_t, src_t, src_val):
    f = interp1d(src_t, src_val, bounds_error=False, fill_value="extrapolate")
    return f(ref_t)


def cut_to_overlap(t_slam, t_dlio, t_lio):
    """Gibt gemeinsames Zeitintervall als (t_min, t_max) zurück."""
    t_min = max(t_slam[0], t_dlio[0], t_lio[0])
    t_max = min(t_slam[-1], t_dlio[-1], t_lio[-1])
    return t_min, t_max


def apply_time_window(t, *arrays, tmin, tmax):
    """Schneidet Zeitreihe + Arrays auf gemeinsames Intervall."""
    mask = (t >= tmin) & (t <= tmax)
    return t[mask], [a[mask] for a in arrays]


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # ------------------------------------
    # CSVs laden
    # ------------------------------------
    slam = pd.read_csv(SLAM_CSV)
    dlio = pd.read_csv(DLIO_CSV)
    lio  = pd.read_csv(LIO_CSV)

    # Zeitachsen extrahieren
    t_slam = slam["time_sec"].values
    t_dlio = dlio["time_sec"].values
    t_lio  = lio["time_sec"].values

    # ------------------------------------
    # Gemeinsames Zeitfenster bestimmen
    # ------------------------------------
    tmin, tmax = cut_to_overlap(t_slam, t_dlio, t_lio)
    print(f"Gemeinsamer Bereich: {tmin:.3f} – {tmax:.3f} (Δ={tmax-tmin:.1f}s)")

    # SLAM Toolbox als Referenzzeit
    t_slam_cut, [px_slam, py_slam] = apply_time_window(
        t_slam,
        slam["px"].values,
        slam["py"].values,
        tmin=tmin, tmax=tmax
    )

    # Zeitachsen normalisieren
    t0 = t_slam_cut[0]
    t_ref = t_slam_cut - t0

    # ------------------------------------
    # DLIO auf Ref synchronisieren
    # ------------------------------------
    px_dlio = interpolate_to(t_slam_cut, t_dlio, dlio["px"].values)
    py_dlio = interpolate_to(t_slam_cut, t_dlio, dlio["py"].values)
    yaw_dlio = quaternion_to_yaw(
        interpolate_to(t_slam_cut, t_dlio, dlio["qz"].values),
        interpolate_to(t_slam_cut, t_dlio, dlio["qw"].values),
    )

    # ------------------------------------
    # LIO-SAM auf Ref synchronisieren
    # ------------------------------------
    px_lio = interpolate_to(t_slam_cut, t_lio, lio["px"].values)
    py_lio = interpolate_to(t_slam_cut, t_lio, lio["py"].values)
    yaw_lio = quaternion_to_yaw(
        interpolate_to(t_slam_cut, t_lio, lio["qz"].values),
        interpolate_to(t_slam_cut, t_lio, lio["qw"].values),
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
    # Fehler SLAM (Ref) vs DLIO/LIO
    # ------------------------------------
    err_dlio = np.sqrt((px_dlio_n - px_slam_n)**2 + (py_dlio_n - py_slam_n)**2)
    err_lio  = np.sqrt((px_lio_n  - px_slam_n)**2 + (py_lio_n - py_slam_n)**2)

    rmse_dlio = np.sqrt(np.mean(err_dlio**2))
    rmse_lio  = np.sqrt(np.mean(err_lio**2))

    # ------------------------------------
    # Ergebnisse speichern
    # ------------------------------------
    with open(os.path.join(OUT_DIR, "comparison_slam_dlio_lio.txt"), "w") as f:
        f.write("=== 2D Vergleich — Referenz: SLAM Toolbox ===\n")
        f.write(f"Zeitbereich: {tmin:.3f} – {tmax:.3f} (Δ={tmax-tmin:.1f}s)\n\n")
        f.write(f"RMSE DLIO   vs SLAM: {rmse_dlio:.3f} m\n")
        f.write(f"RMSE LIO-SAM vs SLAM: {rmse_lio:.3f} m\n")

    # ------------------------------------
    # Plots
    # ------------------------------------
    # Trajektorien
    plt.figure(figsize=(8,6))
    plt.plot(px_slam_n, py_slam_n, label="SLAM Toolbox (Ref)", linewidth=2)
    plt.plot(px_dlio_n, py_dlio_n, label="DLIO")
    plt.plot(px_lio_n,  py_lio_n,  label="LIO-SAM")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.axis("equal")
    plt.legend()
    plt.title("2D Trajectories (overlap only)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "traj_2d_comparison.png"))
    plt.close()

    # Fehlerplot
    plt.figure(figsize=(8,5))
    plt.plot(t_ref, err_dlio, label="DLIO Error")
    plt.plot(t_ref, err_lio, label="LIO-SAM Error")
    plt.xlabel("time [s]")
    plt.ylabel("XY error [m]")
    plt.legend()
    plt.title("XY Error Over Time (overlap only)")
    plt.tight_layout()
    plt.savefig(os.path.join(OUT_DIR, "error_over_time.png"))
    plt.close()

    print("Vergleich abgeschlossen.")
    print("→ Output Ordner:", OUT_DIR)


if __name__ == "__main__":
    main()
