# error_correlations.py
import numpy as np
import matplotlib.pyplot as plt
from accessible_plots import plot_accessible_hexbin

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
