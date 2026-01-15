#!/usr/bin/env python3
"""
Dieses Skript führt eine vollständige 2D-Vergleichsanalyse zwischen DLIO und LIO-SAM durch.
Es nutzt die DLIO-Pose als Referenzzeitskala und interpoliert die LIO-SAM-Pose darauf.

Funktionen des Skripts:

1) Daten laden:
   - DLIO: automatisch die neueste *_pose_logger_output.csv aus /data
   - LIO-SAM: feste Datei /data/lio_sam.csv

2) Zeit-Synchronisation:
   - Normiert beide Zeitachsen auf t=0
   - Interpoliert alle LIO-SAM-Positions- und Quaternionwerte auf die DLIO-Zeitstempel

3) Trajektorienvergleich:
   - Normalisiert alle Trajektorien auf denselben Startpunkt
   - Berechnet XY-Fehler, RMSE, mittleren Fehler, maximalen Fehler
   - Erzeugt 2D-Trajektorieplots, Fehlerverläufe, Histogramme, CDF, XY-Fehlerheatmaps

4) Orientierung & Drift:
   - Wandelt Quaternionen in Yaw um
   - Berechnet Yaw-Drift zwischen DLIO und LIO-SAM
   - Erstellt Yaw-Drift-Plots und Korrelationen

5) IMU-basierte Fehleranalyse:
   - Berechnet Geschwindigkeit, Beschleunigung und Gyro-Z-Werte aus DLIO
   - Korrelierte Plots: Fehler vs. Beschleunigung, Gyro, Geschwindigkeit

6) Erweiterte Diagnostik:
   - Fehler vs. Yaw
   - Yaw-Fehler-Heatmaps
   - Fehlerverteilung (Violin-Plot)

Alle erzeugten Grafiken und Ergebnisse werden unter /output gespeichert.
Das Skript liefert eine vollständige 2D-Analyse der Unterschiede zwischen DLIO und LIO-SAM.
"""

import os
import pandas as pd
import numpy as np
from glob import glob
import matplotlib.pyplot as plt
from scipy.stats import linregress

from accessible_plots import (
    plot_accessible,
    plot_histogram,
    plot_cdf,
    plot_xy_error_heatmap,
    quaternion_to_yaw,
    plot_yaw_error,
    plot_error_vs_imu,
    plot_accessible_hexbin,
    plot_error_vs_yaw,
    plot_yaw_heatmap,
    plot_yaw_derivative_vs_error,
    plot_error_violin
)


# ======================================================
# Pfade
# ======================================================
DLIO_DIR    = "/data"
LIO_SAM_CSV = "/data/lio_sam.csv"
OUT_DIR     = "/output"


# ======================================================
# Datei finden
# ======================================================
def find_latest_dlio_csv():
    files = sorted(glob(os.path.join(DLIO_DIR, "*_pose_logger_output.csv")))
    if len(files) == 0:
        raise FileNotFoundError("Keine DLIO-Datei gefunden!")
    return files[-1]


# ======================================================
# Laden + Vorbereiten
# ======================================================
def load_and_prepare():
    print("Lade Dateien…")

    dlio_path = find_latest_dlio_csv()
    lio = pd.read_csv(LIO_SAM_CSV)
    dlio = pd.read_csv(dlio_path)

    dlio = dlio[(dlio["px"].abs() > 1e-6) | (dlio["py"].abs() > 1e-6)].copy()
    dlio = dlio.reset_index(drop=True)

    if len(dlio) < 5:
        raise ValueError("Zu wenige DLIO-Datenpunkte!")

    return dlio, lio


# ======================================================
# Zeit-Synchronisation
# ======================================================
def interpolate_lio_to_dlio(dlio, lio):
    print("Interpoliere LIO-SAM → DLIO-Zeit…")

    t_dlio = dlio["time_sec"].values
    t_lio  = lio["time_sec"].values

    # Zeit normalisieren
    t_dlio = t_dlio - t_dlio[0]
    t_lio  = t_lio  - t_lio[0]

    # Interpolation der relevanten Größen
    lio_interp = {
        ax: np.interp(t_dlio, t_lio, lio[ax].values)
        for ax in ["px", "py", "pz", "qx", "qy", "qz", "qw"]
    }

    return (
        t_dlio,
        dlio["px"].values.copy(),
        dlio["py"].values.copy(),
        lio_interp
    )

# ======================================================
# Normalisieren
# ======================================================
def normalize(px, py):
    return px - px[0], py - py[0]


# ======================================================
# Fehler berechnen
# ======================================================
def compute_errors(px_dlio, py_dlio, px_lio, py_lio):
    err_x = px_dlio - px_lio
    err_y = py_dlio - py_lio

    err_norm = np.sqrt(err_x**2 + err_y**2)

    rmse = np.sqrt(np.mean(err_norm**2))
    mean_err = np.mean(err_norm)
    max_err  = np.max(err_norm)

    sorted_err = np.sort(err_norm)
    cdf = np.arange(len(sorted_err)) / len(sorted_err)

    return err_x, err_y, err_norm, sorted_err, cdf, rmse, mean_err, max_err


# ======================================================
# Ergebnisse speichern
# ======================================================
def save_results(rmse, mean_err, max_err):
    with open(os.path.join(OUT_DIR, "comparison_results_2d.txt"), "w") as f:
        f.write(f"RMSE XY (m): {rmse:.3f}\n")
        f.write(f"Mean XY Error (m): {mean_err:.3f}\n")
        f.write(f"Max XY Error (m): {max_err:.3f}\n")

    print("Ergebnisse gespeichert.")

# ======================================================
# Error correlations
# ======================================================

def plot_error_vs_acc(t, error, acc, save_path):
    plot_accessible_hexbin(
        x=acc,
        y=error,
        xlabel="Acceleration |a| [m/s²]",
        ylabel="XY Error [m]",
        title="Error vs. Acceleration",
        save_path=save_path
    )


def plot_error_vs_gyro(t, error, gyro, save_path):
    plot_accessible_hexbin(
        x=np.abs(gyro),
        y=error,
        xlabel="|Gyro_z| [rad/s]",
        ylabel="XY Error [m]",
        title="Error vs. Gyro Rate",
        save_path=save_path
    )


def plot_error_vs_velocity(t, error, vel, save_path):
    plot_accessible_hexbin(
        x=vel,
        y=error,
        xlabel="Velocity |v| [m/s]",
        ylabel="XY Error [m]",
        title="Error vs. Velocity",
        save_path=save_path
    )

# ======================================================
# Motion metrics
# ======================================================

def compute_velocity(dlio):
    vx = dlio["vx"].values
    vy = dlio["vy"].values
    vz = dlio["vz"].values
    vel_mag = np.sqrt(vx**2 + vy**2 + vz**2)
    return vel_mag


def compute_acceleration(dlio):
    ax = dlio["wx"].values * 0
    ay = dlio["wy"].values * 0
    az = dlio["wz"].values * 0

    if all(k in dlio.columns for k in ["ax", "ay", "az"]):
        ax = dlio["ax"].values
        ay = dlio["ay"].values
        az = dlio["az"].values

    acc_mag = np.sqrt(ax**2 + ay**2 + az**2)
    return acc_mag


def compute_gyro_z(dlio):
    return dlio["wz"].values

# ======================================================
# MAIN
# ======================================================
def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # Laden
    dlio, lio = load_and_prepare()

    # Interpolation
    t_dlio, px_dlio, py_dlio, lio_interp = interpolate_lio_to_dlio(dlio, lio)

    # Extrahieren der LIO-SAM Werte aus dem Dict
    px_lio = lio_interp["px"].copy()
    py_lio = lio_interp["py"].copy()

    # Normalisieren
    px_dlio, py_dlio = normalize(px_dlio, py_dlio)
    px_lio,  py_lio  = normalize(px_lio,  py_lio)

    # Fehlerberechnung
    (
        err_x, err_y, err_norm,
        sorted_err, cdf,
        rmse, mean_err, max_err
    ) = compute_errors(px_dlio, py_dlio, px_lio, py_lio)

    save_results(rmse, mean_err, max_err)

    # ---------------------------------------------
    # PLOTS
    # ---------------------------------------------

    plot_accessible(
        curves=[
            {"x": px_dlio, "y": py_dlio, "label": "DLIO"},
            {"x": px_lio,  "y": py_lio,  "label": "LIO-SAM"},
        ],
        xlabel="x [m]",
        ylabel="y [m]",
        title="2D Trajectory Comparison (XY)",
        save_path=os.path.join(OUT_DIR, "traj_xy_2d.png"),
        equal_axis=True,
        downsample=20
    )

    plot_accessible(
        curves=[{"x": t_dlio, "y": err_norm, "label": "Error Norm"}],
        xlabel="Time [s]",
        ylabel="XY Error [m]",
        title="2D Position Error Over Time",
        save_path=os.path.join(OUT_DIR, "error_time_2d.png"),
        downsample=20
    )

    plot_accessible(
        curves=[
            {"x": t_dlio, "y": err_x, "label": "err_x"},
            {"x": t_dlio, "y": err_y, "label": "err_y"}
        ],
        xlabel="Time [s]",
        ylabel="Error [m]",
        title="Component-wise XY Error",
        save_path=os.path.join(OUT_DIR, "error_xy.png"),
        downsample=20
    )

    plot_histogram(
        err_norm,
        save_path=os.path.join(OUT_DIR, "hist_xy.png")
    )

    plot_cdf(
        sorted_err,
        cdf,
        save_path=os.path.join(OUT_DIR, "cdf_2d.png")
    )

    # XY Error Heatmap
    plot_xy_error_heatmap(
        px_lio, py_lio, err_norm,
        save_path=os.path.join(OUT_DIR, "xy_error_heatmap.png")
    )

    # Yaw Drift
    plot_yaw_error(
        dlio, lio_interp, t_dlio,
        save_path=os.path.join(OUT_DIR, "yaw_drift.png")
    )

    # Error vs IMU
    plot_error_vs_imu(
        dlio, err_norm, t_dlio,
        save_path=os.path.join(OUT_DIR, "error_vs_imu.png")
    )

    # ----------------------------------------
    # Motion Metrics berechnen
    # ----------------------------------------
    vel_mag = compute_velocity(dlio)
    acc_mag = compute_acceleration(dlio)
    gyro_z  = compute_gyro_z(dlio)

    # Downsample für Heatmaps
    ds = slice(0, len(err_norm), 10)

    # ----------------------------------------
    # Heatmaps / Korrelationen
    # ----------------------------------------
    plot_error_vs_acc(
        t_dlio[ds], err_norm[ds], acc_mag[ds],
        save_path=os.path.join(OUT_DIR, "error_vs_acc.png")
    )

    plot_error_vs_gyro(
        t_dlio[ds], err_norm[ds], gyro_z[ds],
        save_path=os.path.join(OUT_DIR, "error_vs_gyro.png")
    )

    plot_error_vs_velocity(
        t_dlio[ds], err_norm[ds], vel_mag[ds],
        save_path=os.path.join(OUT_DIR, "error_vs_vel.png")
    )
   
    yaw_err = np.unwrap(np.arctan2(
        2*(dlio["qw"]*dlio["qz"] + dlio["qx"]*dlio["qy"]),
        1 - 2*(dlio["qy"]**2 + dlio["qz"]**2)
    )) - np.unwrap(np.arctan2(
        2*(lio_interp["qw"]*lio_interp["qz"] + lio_interp["qx"]*lio_interp["qy"]),
        1 - 2*(lio_interp["qy"]**2 + lio_interp["qz"]**2)
    ))

    plot_error_vs_yaw(t_dlio, err_norm, yaw_err, OUT_DIR)
    plot_yaw_heatmap(err_norm, yaw_err, OUT_DIR)
    plot_yaw_derivative_vs_error(t_dlio, yaw_err, err_norm, OUT_DIR)

    plot_error_violin(
        err_norm,
        save_path=os.path.join(OUT_DIR, "violin_error.png")
    )



    print("Alle 2D Plots gespeichert.")
    print("2D Analyse abgeschlossen.")

# ======================================================
# Script-Start
# ======================================================
if __name__ == "__main__":
    main()
