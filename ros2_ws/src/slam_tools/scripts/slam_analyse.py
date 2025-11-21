#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import glob

# ----------------------------------------------------------
# Pfade fest definiert
# ----------------------------------------------------------

DATA_DIR = "/data"
OUTPUT_DIR = "/home/ubuntu/ros2_ws/src/slam_tools/output"

# Erstelle Output-Ordner falls nicht vorhanden
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ----------------------------------------------------------
# Hilfsfunktionen
# ----------------------------------------------------------

def load_latest_csv(prefix):
    """Findet die neueste CSV aus /data basierend auf Prefix."""
    pattern = os.path.join(DATA_DIR, f"*{prefix}*.csv")
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"Keine CSV mit Prefix '{prefix}' in {DATA_DIR}")
    return files[-1]  # die neuste Datei


def synchronize(pose_df, imu_df, max_dt=0.01):
    """Synchronisiert IMU zu Posen per Nearest Neighbour."""

    imu_times = imu_df["time_sec"].values
    pose_times = pose_df["time_sec"].values

    imu_idx = np.searchsorted(imu_times, pose_times)
    matched = []

    for i, t in enumerate(pose_times):
        candidates = []
        if imu_idx[i] < len(imu_times):
            candidates.append(imu_idx[i])
        if imu_idx[i] > 0:
            candidates.append(imu_idx[i]-1)

        if not candidates:
            matched.append([np.nan]*6)
            continue

        best = min(candidates, key=lambda j: abs(imu_times[j] - t))

        if abs(imu_times[best] - t) > max_dt:
            matched.append([np.nan]*6)
        else:
            row = imu_df.iloc[best][["ax","ay","az","gx","gy","gz"]].values.tolist()
            matched.append(row)

    matched = np.array(matched)
    merged = pose_df.copy()
    merged["ax"] = matched[:,0]
    merged["ay"] = matched[:,1]
    merged["az"] = matched[:,2]
    merged["gx"] = matched[:,3]
    merged["gy"] = matched[:,4]
    merged["gz"] = matched[:,5]

    return merged


def plot_data(df):
    """Erstellt Plots und speichert sie ins OUTPUT_DIR."""
    t = df["time_sec"]

    # Pose
    fig, ax = plt.subplots(3,1, figsize=(10,8), sharex=True)
    ax[0].plot(t, df["px"]); ax[0].set_ylabel("px")
    ax[1].plot(t, df["py"]); ax[1].set_ylabel("py")
    ax[2].plot(t, df["pz"]); ax[2].set_ylabel("pz"); ax[2].set_xlabel("time [s]")
    fig.suptitle("Pose over time")
    fig.tight_layout()
    fig.savefig(f"{OUTPUT_DIR}/pose.png")
    plt.close(fig)

    # Accel
    fig, ax = plt.subplots(3,1, figsize=(10,8), sharex=True)
    ax[0].plot(t, df["ax"]); ax[0].set_ylabel("ax")
    ax[1].plot(t, df["ay"]); ax[1].set_ylabel("ay")
    ax[2].plot(t, df["az"]); ax[2].set_ylabel("az"); ax[2].set_xlabel("time [s]")
    fig.suptitle("IMU acceleration")
    fig.tight_layout()
    fig.savefig(f"{OUTPUT_DIR}/accel.png")
    plt.close(fig)

    # Gyro
    fig, ax = plt.subplots(3,1, figsize=(10,8), sharex=True)
    ax[0].plot(t, df["gx"]); ax[0].set_ylabel("gx")
    ax[1].plot(t, df["gy"]); ax[1].set_ylabel("gy")
    ax[2].plot(t, df["gz"]); ax[2].set_ylabel("gz"); ax[2].set_xlabel("time [s]")
    fig.suptitle("IMU gyro")
    fig.tight_layout()
    fig.savefig(f"{OUTPUT_DIR}/gyro.png")
    plt.close(fig)


def print_stats(df):
    print("\n=== Basis-Statistiken ===")
    print(f"Samples: {len(df)}")
    print(f"Dauer: {df['time_sec'].iloc[-1] - df['time_sec'].iloc[0]:.2f} s")
    print("\nPositions-Std-Abweichung:")
    print(df[["px","py","pz"]].std())
    print("\nIMU-Std-Abweichung:")
    print(df[["ax","ay","az","gx","gy","gz"]].std())


# ----------------------------------------------------------
# Hauptfunktion
# ----------------------------------------------------------

def main():

    print("[INFO] Suche CSV-Dateien in /data ...")
    pose_file = load_latest_csv("pose")
    imu_file  = load_latest_csv("imu")

    print(f"[INFO] Lade Pose-CSV: {pose_file}")
    pose_df = pd.read_csv(pose_file)

    print(f"[INFO] Lade IMU-CSV:  {imu_file}")
    imu_df  = pd.read_csv(imu_file)

    print("[INFO] Synchronisiere Daten…")
    merged = synchronize(pose_df, imu_df)

    out_csv = f"{OUTPUT_DIR}/merged.csv"
    merged.to_csv(out_csv, index=False)
    print(f"[OK] Gespeichert: {out_csv}")

    print("[INFO] Erzeuge Plots…")
    plot_data(merged)

    print_stats(merged)
    print("[OK] Fertig.")


if __name__ == "__main__":
    main()
