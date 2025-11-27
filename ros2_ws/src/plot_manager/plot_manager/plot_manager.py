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
    plot_accessible_hexbin,
    plot_error_vs_yaw,
    plot_yaw_heatmap,
    plot_yaw_derivative_vs_error,
    plot_error_violin,
    plot_violin_multi,
    plot_trajectory_colored_by_error
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
    # IMU auf Referenzzeit interpolieren
    # --------------------------------------------------------
    imu_interp = {
        "wx": np.interp(t_ref, t, df["wx"].values),
        "wy": np.interp(t_ref, t, df["wy"].values),
        "wz": np.interp(t_ref, t, df["wz"].values),
        "vx": np.interp(t_ref, t, df["vx"].values),
        "vy": np.interp(t_ref, t, df["vy"].values),
        "vz": np.interp(t_ref, t, df["vz"].values),
    }

    # --------------------------------------------------------
    # 1) Trajektorie des Algorithmus
    # --------------------------------------------------------
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

    # --------------------------------------------------------
    # 2) Error über Zeit
    # --------------------------------------------------------
    plot_error_over_time_multi(
        t_ref,
        {name: err_xy},
        f"{out}/error_over_time.png"
    )

    # --------------------------------------------------------
    # 3) CDF
    # --------------------------------------------------------
    plot_cdf_multi({name: err_xy}, f"{out}/cdf.png")

    # --------------------------------------------------------
    # 4) Histogramm
    # --------------------------------------------------------
    plot_histogram_multi({name: err_xy}, f"{out}/hist.png")

    # --------------------------------------------------------
    # 5) Violin
    # --------------------------------------------------------
    plot_violin_multi({name: err_xy}, f"{out}/violin.png")

    # --------------------------------------------------------
    # 6) Trajektorie gefärbt nach Fehler
    # --------------------------------------------------------
    plot_trajectory_colored_by_error(
        interp["px"],
        interp["py"],
        err_xy,
        f"{out}/traj_error.png"
    )

    # --------------------------------------------------------
    # 7) Heatmap
    # --------------------------------------------------------
    plot_xy_error_heatmap(
        interp["px"], interp["py"],
        err_xy,
        f"{out}/xy_error_heatmap.png"
    )

    # --------------------------------------------------------
    # 8) Yaw Error
    # --------------------------------------------------------
    plot_yaw_error(
        ref,   # DataFrame (Referenz)
        interp,  # Dict mit numpy-Arrays
        t_ref,
        f"{out}/yaw_error.png"
    )


    # --------------------------------------------------------
    # 9) IMU + Error (korrekt)
    # --------------------------------------------------------
    plot_error_vs_imu(
        imu_interp,
        err_xy,
        t_ref,      # FIX!!
        f"{out}/imu.png"
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

    # Referenz-Zeit normalisieren
    t_ref = ref["time_sec"].values - ref["time_sec"].values[0]

    names_all = []      # wirklich alle
    names_plot = []     # nur Nicht-Referenz!
    errs_all = {}
    yaw_errs_all = {}
    csv_data = {}

    # --------------------------------------------------------
    # CSVs verarbeiten
    # --------------------------------------------------------
    for path in csv_paths:
        name = os.path.basename(path).replace(".csv", "")
        df = load_csv(path)

        print(f"\n→ Verarbeitung: {name}")

        # -------------------------
        # REFERENZ-Fall
        # -------------------------
        if path == ref_path:
            interp = {
                k: ref[k].values
                for k in ["px","py","pz","qx","qy","qz","qw",
                          "vx","vy","vz","wx","wy","wz"]
            }
            err_xy = np.zeros(len(t_ref))
            yaw_err = np.zeros(len(t_ref))

        # -------------------------
        # VERGLEICHS-Fälle
        # -------------------------
        else:
            t_interp, interp = interpolate_to_reference(ref, df)
            err_xy, yaw_err = compute_errors(ref, interp)
            names_plot.append(name)           # <— nur diese anzeigen!

        # Daten speichern
        names_all.append(name)
        errs_all[name] = err_xy
        yaw_errs_all[name] = yaw_err
        csv_data[name] = {
            "interp": interp,
            "raw": df
        }

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

    # ============================================================
    # 3) Fehlerfarbige Trajektorien
    # ============================================================
    for name in names_all:
        if name == os.path.basename(ref_path).replace(".csv", ""):
            continue  # <-- Referenz überspringen

        px = csv_data[name]["interp"]["px"]
        py = csv_data[name]["interp"]["py"]
        err = errs_all[name]

        plot_trajectory_colored_by_error(
            px, py, err,
            f"{out}/traj_error_{name}.png"
        )


    print(f"\n✓ Multiplots gespeichert unter: {out}")




# ============================================================
# MAIN
# ============================================================
def main():
    print("SLAM Plot Manager")
    print("1 = Single Plot")
    print("2 = Multi Plot")

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


if __name__ == "__main__":
    main()
