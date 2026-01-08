# plot_manager

ROS 2 Python package for **visual comparison and error analysis of SLAM
/ odometry CSV logs**.\
Focus: **barrierefreie, wissenschaftlich saubere Plots** mit
**unterstützter variabler Trajektorienlänge**.

------------------------------------------------------------------------

## Features

-   Vergleich mehrerer SLAM-Algorithmen gegen eine Referenz
-   **Variable-Length Plots** (keine erzwungene Zeitangleichung)
-   Barrierefreie Farbpaletten (Okabe--Ito)
-   Publikationsreife PNG & SVG Outputs
-   Fokus auf:
    -   Trajectories
    -   Error over Time
    -   CDF Comparison
    -   Histogram Comparison

------------------------------------------------------------------------

## Package Structure

    plot_manager/
    ├── plot_manager/
    │   ├── plot_manager.py
    │   ├── helpers.py
    │   ├── accessible_plots.py
    │   └── __init__.py
    ├── package.xml
    ├── setup.py
    └── README.md

------------------------------------------------------------------------

## CSV Format (Minimal)

Pflichtspalten:

    time_sec, px, py, pz, qx, qy, qz, qw

Optionale Spalten (werden automatisch ergänzt):

    vx, vy, vz, wx, wy, wz

------------------------------------------------------------------------

## Plot Pipelines

### run_multi_plot()

-   Interpolation aller Algorithmen auf Referenzzeit
-   Alle Kurven gleich lang

### run_multi_plot_v2() (empfohlen)

-   **Keine Zeitinterpolation**
-   Unterschiedlich lange CSVs bleiben unterschiedlich lang
-   Overlap nur für Fehlerstatistik

Erzeugte Plots: - Trajectory Comparison - Error Over Time - CDF
Comparison - Histogram Comparison

------------------------------------------------------------------------

## Verwendung

``` bash
ros2 run plot_manager plot_manager
```

------------------------------------------------------------------------

## Output

    /output/<timestamp>_Multiplot_v2/

PNG + SVG

------------------------------------------------------------------------

## Abhängigkeiten

-   numpy
-   pandas
-   matplotlib
-   scipy
