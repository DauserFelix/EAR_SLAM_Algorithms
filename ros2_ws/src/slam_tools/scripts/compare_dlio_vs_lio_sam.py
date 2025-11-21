#!/usr/bin/env python3
import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from glob import glob

# ---------------------------------------
# Pfade
# ---------------------------------------
DLIO_DIR = "/data"
LIO_SAM_CSV = "/home/ubuntu/ros2_ws/src/slam_tools/output/lio_sam_gt.csv"
OUT_DIR = "/home/ubuntu/ros2_ws/src/slam_tools/output"

os.makedirs(OUT_DIR, exist_ok=True)


# ---------------------------------------
# Hilfsfunktion: Neuste DLIO-CSV finden
# ---------------------------------------
def find_latest_dlio_csv():
    files = sorted(glob(os.path.join(DLIO_DIR, "*_pose_logger_output.csv")))
    if len(files) == 0:
        raise FileNotFoundError("Keine DLIO-Datei in /data gefunden!")
    return files[-1]


# ---------------------------------------
# CSVs laden
# ---------------------------------------
print("Suche DLIO CSV...")
dlio_path = find_latest_dlio_csv()
print(f"  DLIO: {dlio_path}")

print("Lade LIO-SAM CSV...")
lio_path = LIO_SAM_CSV
print(f"  LIO-SAM: {lio_path}")

dlio = pd.read_csv(dlio_path)
lio  = pd.read_csv(lio_path)

# DLIO Null-Zeilen entfernen (Start)
dlio = dlio[(dlio["px"] != 0) | (dlio["py"] != 0) | (dlio["pz"] != 0)].copy()


# ---------------------------------------
# Zeit synchronisieren
# ---------------------------------------
print("Synchronisiere anhand der Zeit...")

dlio_time = dlio["time_sec"].values
lio_time  = lio["time_sec"].values

# Indexe zuordnen
lio_indices = np.searchsorted(lio_time, dlio_time, side="left")
lio_indices = np.clip(lio_indices, 0, len(lio_time) - 1)

# Gefilterte, synchronisierte Versionen
dlio_sync = dlio.reset_index(drop=True)
lio_sync  = lio.iloc[lio_indices].reset_index(drop=True)


# ---------------------------------------
# Positionsdaten extrahieren
# ---------------------------------------
pos_dlio = dlio_sync[["px", "py", "pz"]].values
pos_lio  = lio_sync[["px", "py", "pz"]].values


# ---------------------------------------
# **Trajektorien auf Startpunkt normalisieren**
# ---------------------------------------
print("Normalisiere Trajektorien (Startpunkt 0/0/0)...")

pos_dlio -= pos_dlio[0]
pos_lio  -= pos_lio[0]


# ---------------------------------------
# Fehler berechnen
# ---------------------------------------
errors = pos_dlio - pos_lio
error_norm = np.linalg.norm(errors, axis=1)

rmse = np.sqrt(np.mean(error_norm**2))
mean_err = np.mean(error_norm)
max_err = np.max(error_norm)


# ---------------------------------------
# Speichern der Ergebnisse
# ---------------------------------------
result_path = os.path.join(OUT_DIR, "comparison_results.txt")
with open(result_path, "w") as f:
    f.write(f"RMSE Position (m): {rmse:.3f}\n")
    f.write(f"Mean Absolute Position Error (m): {mean_err:.3f}\n")
    f.write(f"Max Position Error (m): {max_err:.3f}\n")

print(f"Ergebnisse gespeichert unter: {result_path}")


# ---------------------------------------
# Plot Trajektorien
# ---------------------------------------
plt.figure(figsize=(8, 6))
plt.plot(pos_dlio[:,0], pos_dlio[:,1], label="DLIO", linewidth=2)
plt.plot(pos_lio[:,0], pos_lio[:,1], label="LIO-SAM GT", linewidth=2)
plt.legend()
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory Comparison (Aligned)")
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "trajectory_comparison.png"), dpi=200)
plt.close()


# ---------------------------------------
# Fehlerplot
# ---------------------------------------
plt.figure(figsize=(8, 6))
plt.plot(error_norm)
plt.xlabel("Index")
plt.ylabel("Positional Error [m]")
plt.title("DLIO vs LIO-SAM Error Over Time (Aligned)")
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "error_over_time.png"), dpi=200)
plt.close()

print("Plots gespeichert!")
print("Analyse fertig.")
