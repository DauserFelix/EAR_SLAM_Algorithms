# plot_manager.py
import os
import pandas as pd
import numpy as np
from datetime import datetime
base_dir = os.path.dirname(os.path.abspath(__file__))

from helpers import (
    load_csv,
    interpolate_to_reference,
    quaternion_to_yaw,
    compute_errors,
    make_output_folder,
    list_csv_files
)

from accessible_plots import (
    plot_accessible,
    plot_cdf_multi,
    plot_error_over_time_multi_varlen,
)

def run_multi_plot_v2(csv_paths):
    """
    Multi Plot v2 (variable Länge):
    - nimmt die erste CSV als Referenz
    - berechnet XY-Fehler nur im zeitlichen Overlap
    - keine Interpolation (indexweise über gleiche Länge nach Overlap-Cut)
    - erzeugt: Trajektorienvergleich, Error-over-Time (varlen), CDF
    """
    print("\n=== MULTI PLOT v2 (variable Länge) ==")

    # Output-Ordner lokal im Projekt
    out = make_output_folder("Multiplot_v2", base_dir)

    # --------------------------------------------------------
    # Referenz laden (nur für Fehlerberechnung)
    # --------------------------------------------------------
    ref_path = csv_paths[0]
    ref = load_csv(ref_path)
    t_ref = ref["time_sec"].values

    traj_curves = []
    errors_stat = {}   # für CDF
    errors_time = {}   # für Error-over-Time (variable Länge)

    # --------------------------------------------------------
    # CSVs verarbeiten
    # --------------------------------------------------------
    for path in csv_paths:
        name = os.path.basename(path).replace(".csv", "")
        df = load_csv(path)

        print(f"→ Verarbeitung: {name}")

        # ----------------------------
        # Trajektorie (RAW)
        # ----------------------------
        traj_curves.append({
            "x": df["px"].values,
            "y": df["py"].values,
            "label": name
        })

        # Referenz selbst: kein Fehler
        if path == ref_path:
            continue

        # ----------------------------
        # Zeit-Overlap bestimmen
        # ----------------------------
        t_algo = df["time_sec"].values
        t_end = min(t_ref[-1], t_algo[-1])

        mask_ref = t_ref <= t_end
        mask_algo = t_algo <= t_end

        ref_cut = ref.loc[mask_ref].reset_index(drop=True)
        algo_cut = df.loc[mask_algo].reset_index(drop=True)

        # Zeit auf gemeinsamen Start normieren (nur lokal)
        t_err = algo_cut["time_sec"].values - algo_cut["time_sec"].values[0]

        # ----------------------------
        # Fehler MIT Zeit-Matching (Interpolation)
        # ----------------------------

        # 1) gemeinsamen Overlap bestimmen (Start + Ende!)
        t0 = max(ref["time_sec"].iloc[0], df["time_sec"].iloc[0])
        t1 = min(ref["time_sec"].iloc[-1], df["time_sec"].iloc[-1])

        ref_ol = ref[(ref["time_sec"] >= t0) & (ref["time_sec"] <= t1)].reset_index(drop=True)
        algo_ol = df[(df["time_sec"] >= t0) & (df["time_sec"] <= t1)].reset_index(drop=True)

        t_ref_ol = ref_ol["time_sec"].values
        t_algo_ol = algo_ol["time_sec"].values

        # Sicherheit: falls zu kurz
        if len(ref_ol) < 5 or len(algo_ol) < 5:
            print(f"⚠️ Skipping {name}: zu wenig Overlap")
            continue

        # 2) Algo-Position auf Referenzzeit interpolieren
        px_i = np.interp(t_ref_ol, t_algo_ol, algo_ol["px"].values)
        py_i = np.interp(t_ref_ol, t_algo_ol, algo_ol["py"].values)

        ref_xy = np.c_[ref_ol["px"].values, ref_ol["py"].values]
        algo_xy = np.c_[px_i, py_i]

        # 3) optional: Startpunkt-Offset entfernen (wie du es wolltest)
        ref_xy = ref_xy - ref_xy[0]
        algo_xy = algo_xy - algo_xy[0]

        err_xy = np.linalg.norm(algo_xy - ref_xy, axis=1)

        # Zeitachse für Plot: auf gemeinsamen Start normieren
        t_err = t_ref_ol - t_ref_ol[0]

        errors_stat[name] = err_xy
        errors_time[name] = {"t": t_err, "err": err_xy}

    # --------------------------------------------------------
    # 1) Trajektorienvergleich (variable Länge)
    # --------------------------------------------------------
    plot_accessible(
        traj_curves,
        xlabel="x [m]",
        ylabel="y [m]",
        title="Trajectory Comparison",
        save_path=f"{out}/trajectories.png",
        equal_axis=True
    )

    # --------------------------------------------------------
    # 2) Error Over Time (variable Länge)
    # --------------------------------------------------------
    plot_error_over_time_multi_varlen(
        errors_time,
        f"{out}/error_over_time.png"
    )

    # --------------------------------------------------------
    # 3) CDF Comparison
    # --------------------------------------------------------
    plot_cdf_multi(
        errors_stat,
        f"{out}/cdf_comparison.png"
    )

    print(f"\n✓ Multiplot v2 gespeichert unter: {out}")
    return errors_time

def print_mean_errors(errors_time):
    print("\nMean XY Error over Time:")
    for name in ["dlio", "fastlio", "lego_loam"]:
        if name in errors_time:
            mean_err = np.mean(errors_time[name]["err"])
            print(f"  {name}: {mean_err:.3f} m")
            

# ============================================================
# MAIN
# ============================================================
def main():
    print("SLAM Plot Manager – Multi Plot v2")

    csv_dir = os.path.join(base_dir, "csv")
    files = list_csv_files(csv_dir)

    # Automatisch lio_sam als Referenz, dann alle anderen
    ref_idx = None
    for i, f in enumerate(files):
        if "lio_sam" in f:
            ref_idx = i
            break
    
    if ref_idx is None:
        print("⚠️ lio_sam.csv nicht gefunden!")
        return
    
    selected = [ref_idx] + [i for i in range(len(files)) if i != ref_idx]
    print(f"Referenz: {files[ref_idx]}")
    print(f"Weitere: {[files[i] for i in selected[1:]]}")

    paths = [os.path.join(csv_dir, files[i]) for i in selected]
    errors_time = run_multi_plot_v2(paths)
    print_mean_errors(errors_time)

if __name__ == "__main__":
    main()
