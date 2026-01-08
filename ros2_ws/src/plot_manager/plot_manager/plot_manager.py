# plot_manager.py
import os
import pandas as pd
import numpy as np
from datetime import datetime

from plot_manager.helpers import (
    load_csv,
    interpolate_to_reference,
    quaternion_to_yaw,
    compute_errors,
    make_output_folder,
    list_csv_files
)

from plot_manager.accessible_plots import (
    plot_accessible,
    plot_histogram,
    plot_histogram_multi,
    plot_cdf,
    plot_cdf_multi,
    plot_xy_error_heatmap,
    plot_yaw_error,
    plot_error_vs_imu,
    plot_error_over_time_multi,
    plot_error_vs_yaw,
    plot_yaw_heatmap,
    plot_yaw_derivative_vs_error,
    plot_error_violin,
    plot_violin_multi,
    plot_trajectory_colored_by_error,
    plot_trajectory_with_frames,
    plot_error_over_time_multi_varlen
)


# ============================================================
# SINGLE PLOT PIPELINE
# ============================================================
def run_single_plot_with_reference(csv_path, ref_path):
    print("\n=== SINGLE PLOT (mit Referenz) ===")

    out = make_output_folder("Singleplot")

    # --------------------------------------------------------
    # CSV laden
    # --------------------------------------------------------
    df = load_csv(csv_path)
    ref = load_csv(ref_path)

    name = os.path.basename(csv_path).replace(".csv", "")

    # Zeitbasis normalisieren
    t_ref = ref["time_sec"].values - ref["time_sec"].values[0]
    t = df["time_sec"].values - df["time_sec"].values[0]

    # --------------------------------------------------------
    # Interpolation auf Referenzzeit
    # --------------------------------------------------------
    t_interp, interp = interpolate_to_reference(ref, df)

    # Fehler
    err_xy, yaw_err = compute_errors(ref, interp)

    # --------------------------------------------------------
    # IMU interpolieren
    # --------------------------------------------------------
    imu_interp_df = pd.DataFrame({
        "wx": np.interp(t_ref, t, df["wx"].values),
        "wy": np.interp(t_ref, t, df["wy"].values),
        "wz": np.interp(t_ref, t, df["wz"].values),
        "vx": np.interp(t_ref, t, df["vx"].values),
        "vy": np.interp(t_ref, t, df["vy"].values),
        "vz": np.interp(t_ref, t, df["vz"].values),
    })

    # ========================================================
    # 1) Trajektorie
    # ========================================================
    plot_accessible(
        curves=[{
            "x": interp["px"],
            "y": interp["py"],
            "label": name
        }],
        xlabel="x [m]",
        ylabel="y [m]",
        title=f"Trajectory ({name})",
        save_path=f"{out}/trajectory.png",
        equal_axis=True
    )

    # ========================================================
    # 2) Error über Zeit
    # ========================================================
    plot_error_over_time_multi(
        t_ref,
        {name: err_xy},
        f"{out}/error_over_time.png"
    )

    # ========================================================
    # 3) CDF
    # ========================================================
    plot_cdf_multi({name: err_xy}, f"{out}/cdf.png")

    # ========================================================
    # 4) Histogramm
    # ========================================================
    plot_histogram_multi({name: err_xy}, f"{out}/hist.png")

    # ========================================================
    # 5) Violin
    # ========================================================
    plot_violin_multi({name: err_xy}, f"{out}/violin.png")

    # ========================================================
    # 6) Fehlergefärbte Trajektorie (2D)
    # ========================================================
    plot_trajectory_colored_by_error(
        interp["px"],
        interp["py"],
        err_xy,
        f"{out}/traj_error.png"
    )

    # ========================================================
    # 7) Heatmap
    # ========================================================
    plot_xy_error_heatmap(
        interp["px"], interp["py"],
        err_xy,
        f"{out}/xy_error_heatmap.png"
    )

    # ========================================================
    # 8) Yaw Fehler
    # ========================================================
    plot_yaw_error(
        ref,        # DataFrame
        interp,     # dict
        t_ref,
        f"{out}/yaw_error.png"
    )

    # ========================================================
    # 9) IMU vs Fehler
    # ========================================================
    plot_error_vs_imu(
        imu_interp_df,     # <-- jetzt korrekt: DataFrame
        err_xy,
        t_ref,
        f"{out}/imu.png"
    )

    # ========================================================
    # 10) Yaw vs Error Scatter + Regression
    # ========================================================
    plot_error_vs_yaw(
        t_ref,
        err_xy,
        yaw_err,
        out
    )

    # ========================================================
    # 11) Yaw vs Error Heatmap
    # ========================================================
    plot_yaw_heatmap(
        err_xy,
        yaw_err,
        out
    )

    # ========================================================
    # 12) dYaw/dt vs Error
    # ========================================================
    plot_yaw_derivative_vs_error(
        t_ref,
        yaw_err,
        err_xy,
        out
    )

    # ========================================================
    # 13) 3D Trajectory with Frames (barrierefrei)
    # ========================================================
    plot_trajectory_with_frames(
        interp["px"], interp["py"], interp["pz"],
        interp["qx"], interp["qy"], interp["qz"], interp["qw"],
        x_ref=ref["px"].values,
        y_ref=ref["py"].values,
        z_ref=ref["pz"].values,
        save_path=f"{out}/trajectory_3d_{name}.png"
    )

    print(f"✓ Singleplots gespeichert unter: {out}")



# ============================================================
# MULTI PLOT PIPELINE
# ============================================================
def run_multi_plot(csv_paths):
    print("\n=== MULTI PLOT ===")
    out = make_output_folder("Multiplot")

    # --------------------------------------------------------
    # Referenz laden
    # --------------------------------------------------------
    ref_path = csv_paths[0]
    ref = load_csv(ref_path)

    # Zeit normalisieren
    t_ref = ref["time_sec"].values - ref["time_sec"].values[0]

    names_all = []      # alle CSV-Namen
    names_plot = []     # nur Nicht-Referenz
    errs_all = {}
    yaw_errs_all = {}
    csv_data = {}
    interpolations = {}  # <--- FEHLTE

    # --------------------------------------------------------
    # CSVs verarbeiten
    # --------------------------------------------------------
    for path in csv_paths:
        name = os.path.basename(path).replace(".csv", "")
        df = load_csv(path)

        print(f"\n→ Verarbeitung: {name}")

        # REFERENZ
        if path == ref_path:
            interp = {
                k: ref[k].values
                for k in ["px","py","pz","qx","qy","qz","qw",
                          "vx","vy","vz","wx","wy","wz"]
            }
            err_xy = np.zeros(len(t_ref))
            yaw_err = np.zeros(len(t_ref))

        # VERGLEICHS-DATEN
        else:
            t_interp, interp = interpolate_to_reference(ref, df)
            err_xy, yaw_err = compute_errors(ref, interp)
            names_plot.append(name)

        # Speichern
        names_all.append(name)
        errs_all[name] = err_xy
        yaw_errs_all[name] = yaw_err
        csv_data[name] = {"interp": interp, "raw": df}
        interpolations[name] = interp     # <--- jetzt korrekt

    # --------------------------------------------------------
    # 1) Trajektorienvergleich (ALLE)
    # --------------------------------------------------------
    curves = []
    for name in names_all:
        curves.append({
            "x": csv_data[name]["interp"]["px"],
            "y": csv_data[name]["interp"]["py"],
            "label": name
        })

    plot_accessible(
        curves,
        "x [m]", "y [m]",
        "Trajectory Comparison",
        f"{out}/trajectories.png",
        equal_axis=True
    )

    # --------------------------------------------------------
    # 2) Vergleichsplots OHNE Referenz
    # --------------------------------------------------------
    errs_no_ref = {name: errs_all[name] for name in names_plot}

    plot_cdf_multi(errs_no_ref, f"{out}/cdf_comparison.png")
    plot_histogram_multi(errs_no_ref, f"{out}/hist_comparison.png")
    plot_violin_multi(errs_no_ref, f"{out}/violin_comparison.png")
    plot_error_over_time_multi(t_ref, errs_no_ref, f"{out}/error_over_time.png")

    # --------------------------------------------------------
    # 3) Fehlerfarbige Trajektorien
    # --------------------------------------------------------
    for name in names_all:
        if name == names_all[0]:  # Referenz
            continue

        px = csv_data[name]["interp"]["px"]
        py = csv_data[name]["interp"]["py"]
        err = errs_all[name]

        plot_trajectory_colored_by_error(
            px, py, err,
            f"{out}/traj_error_{name}.png"
        )

    print(f"\n✓ Multiplots gespeichert unter: {out}")

def run_multi_plot_v2(csv_paths):
    print("\n=== MULTI PLOT v2 (variable Länge) ===")
    out = make_output_folder("Multiplot_v2")

    # --------------------------------------------------------
    # Referenz laden (nur für Fehlerberechnung)
    # --------------------------------------------------------
    ref_path = csv_paths[0]
    ref = load_csv(ref_path)
    t_ref = ref["time_sec"].values
    t_ref0 = t_ref[0]

    names_all = []
    names_plot = []
    traj_curves = []
    errors_stat = {}      # für CDF / Histogram
    errors_time = {}      # für Error-over-Time (variable Länge)

    # --------------------------------------------------------
    # CSVs verarbeiten
    # --------------------------------------------------------
    for path in csv_paths:
        name = os.path.basename(path).replace(".csv", "")
        df = load_csv(path)

        print(f"→ Verarbeitung: {name}")
        names_all.append(name)

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

        names_plot.append(name)

        # ----------------------------
        # Zeit-Overlap bestimmen
        # ----------------------------
        t_algo = df["time_sec"].values
        t_end = min(t_ref[-1], t_algo[-1])

        mask_ref  = t_ref  <= t_end
        mask_algo = t_algo <= t_end

        ref_cut  = ref.loc[mask_ref].reset_index(drop=True)
        algo_cut = df.loc[mask_algo].reset_index(drop=True)

        # Zeit auf gemeinsamen Start normieren (nur lokal)
        t_err = algo_cut["time_sec"].values - algo_cut["time_sec"].values[0]

        # ----------------------------
        # Fehler OHNE Interpolation
        # (Indexweise, gleiche Länge durch Overlap)
        # ----------------------------
        N = min(len(ref_cut), len(algo_cut))
        ref_cut  = ref_cut.iloc[:N]
        algo_cut = algo_cut.iloc[:N]
        t_err    = t_err[:N]

        ex = algo_cut["px"].values - ref_cut["px"].values
        ey = algo_cut["py"].values - ref_cut["py"].values
        err_xy = np.sqrt(ex**2 + ey**2)

        errors_stat[name] = err_xy
        errors_time[name] = {
            "t": t_err,
            "err": err_xy
        }

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
    plot_error_over_time_multi_varlen(errors_time,
        f"{out}/error_over_time.png"
    )


    # --------------------------------------------------------
    # 3) CDF Comparison
    # --------------------------------------------------------
    plot_cdf_multi(errors_stat, f"{out}/cdf_comparison.png")

    # --------------------------------------------------------
    # 4) Histogram Comparison
    # --------------------------------------------------------
    plot_histogram_multi(errors_stat, f"{out}/hist_comparison.png")

    print(f"\n✓ Multiplot v2 gespeichert unter: {out}")




# ============================================================
# MAIN
# ============================================================
def main():
    print("SLAM Plot Manager")
    print("1 = Single Plot")
    print("2 = Multi Plot")
    print("3 = Multi Plot v2")

    mode = int(input("Auswahl: "))

    files = list_csv_files()

    if mode == 1:
        print("Wähle CSV:")
        algo = int(input("Algorithmus: "))
        print("Wähle Referenz:")
        ref = int(input("Referenz: "))
        run_single_plot_with_reference(
            os.path.join("/data", files[algo]),
            os.path.join("/data", files[ref])
        )


    elif mode == 2:
        print("Wähle erste CSV als Referenz:")
        ref = int(input("Referenz: "))

        selected = [ref]
        while True:
            s = input("Weitere CSV auswählen (ENTER zum Beenden): ")
            if s == "":
                break
            selected.append(int(s))

        paths = [os.path.join("/data", files[i]) for i in selected]
        run_multi_plot(paths)

    elif mode == 3:
        print("Wähle erste CSV als Referenz:")
        ref = int(input("Referenz: "))

        selected = [ref]
        while True:
            s = input("Weitere CSV auswählen (ENTER zum Beenden): ")
            if s == "":
                break
            selected.append(int(s))

        paths = [os.path.join("/data", files[i]) for i in selected]
        run_multi_plot_v2(paths)


if __name__ == "__main__":
    main()
