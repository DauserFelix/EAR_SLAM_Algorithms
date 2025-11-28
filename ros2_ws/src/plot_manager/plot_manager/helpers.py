import os
import pandas as pd
import numpy as np
from datetime import datetime

# ============================================================
# Hilfsfunktion: CSV laden
# ============================================================
def load_csv(path):
    df = pd.read_csv(path)

    # ---------------------------------------------
    # MINIMALFORMAT AKZEPTIEREN
    # time_sec,px,py,pz,qx,qy,qz,qw
    # ---------------------------------------------
    minimal_cols = ["time_sec", "px", "py", "pz", "qx", "qy", "qz", "qw"]

    if all(col in df.columns for col in minimal_cols):

        # Fehlende IMU/Velocity-Spalten generieren
        missing_cols = ["vx", "vy", "vz", "wx", "wy", "wz"]
        for col in missing_cols:
            if col not in df.columns:
                df[col] = 0.0

    # ---------------------------------------------
    # 1) Leere Zeilen entfernen
    # ---------------------------------------------
    df = df.dropna(how="all")

    # ---------------------------------------------
    # 2) Numerische Spalten finden
    # ---------------------------------------------
    num_cols = df.select_dtypes(include=[np.number]).columns.tolist()

    if len(num_cols) == 0:
        return df.reset_index(drop=True)

    # ---------------------------------------------
    # 3) Komplett-Null-Zeilen entfernen
    # ---------------------------------------------
    mask_not_all_zero = ~(df[num_cols].abs().sum(axis=1) < 1e-12)
    df = df[mask_not_all_zero]

    # ---------------------------------------------
    # 4) Zeitspalte konsistent halten
    # ---------------------------------------------
    time_candidates = [c for c in df.columns if "time" in c.lower()]

    if len(time_candidates) > 0:
        tcol = time_candidates[0]

        t = df[tcol].values
        keep = np.insert(np.diff(t) >= 0, 0, True)
        df = df[keep]

    df = df.reset_index(drop=True)

    if len(df) < 5:
        raise ValueError(f"Zu wenige brauchbare Datenpunkte in {path}")

    return df


# ============================================================
# Interpolation: Algorithmus → Referenzzeit
# ============================================================
def interpolate_to_reference(ref, algo):
    print(f"→ Interpoliere {algo} auf Referenzzeit...")

    t_ref = ref["time_sec"].values
    t_algo = algo["time_sec"].values

    # Zeit normalisieren
    t_ref = t_ref - t_ref[0]
    t_algo = t_algo - t_algo[0]

    interp = {
        key: np.interp(t_ref, t_algo, algo[key].values)
        for key in ["px", "py", "pz", "qx", "qy", "qz", "qw",
                    "vx", "vy", "vz", "wx", "wy", "wz"]
    }

    return t_ref, interp

# ============================================================
# Fehlerberechnung (XY + Yaw)
# ============================================================
def quaternion_to_yaw(qx, qy, qz, qw):
    return np.arctan2(
        2*(qw*qz + qx*qy),
        1 - 2*(qy*qy + qz*qz)
    )

def compute_errors(ref, interp):
    # Position
    ex = interp["px"] - ref["px"].values
    ey = interp["py"] - ref["py"].values
    err_xy = np.sqrt(ex**2 + ey**2)

    # Yaw
    yaw_ref = quaternion_to_yaw(
        ref["qx"].values, ref["qy"].values,
        ref["qz"].values, ref["qw"].values
    )

    yaw_algo = quaternion_to_yaw(
        interp["qx"], interp["qy"], interp["qz"], interp["qw"]
    )
    yaw_err = np.unwrap(yaw_algo - yaw_ref)

    return err_xy, yaw_err

# ============================================================
# Output-Ordner erstellen
# ============================================================
def make_output_folder(mode):
    ts = datetime.now().strftime("%m-%d_%H-%M")
    folder = f"/output/{ts}_{mode}"
    os.makedirs(folder, exist_ok=True)
    return folder


# ============================================================
# MENÜ
# ============================================================
def list_csv_files():
    files = [f for f in os.listdir("/data") if f.endswith(".csv")]
    for i, f in enumerate(files):
        print(f"[{i}] {f}")
    return files


# ============================================================
# Quaternion → Rotationsmatrix
# ============================================================
def quat_to_rot(qx, qy, qz, qw):
    R = np.zeros((3, 3))
    R[0,0] = 1 - 2*(qy*qy + qz*qz)
    R[0,1] = 2*(qx*qy - qz*qw)
    R[0,2] = 2*(qx*qz + qy*qw)
    R[1,0] = 2*(qx*qy + qz*qw)
    R[1,1] = 1 - 2*(qx*qx + qz*qz)
    R[1,2] = 2*(qy*qz - qx*qw)
    R[2,0] = 2*(qx*qz - qy*qw)
    R[2,1] = 2*(qy*qz + qx*qw)
    R[2,2] = 1 - 2*(qx*qx + qy*qy)
    return R