# motion_metrics.py
import numpy as np

def compute_velocity(dlio):
    vx = dlio["vx"].values
    vy = dlio["vy"].values
    vz = dlio["vz"].values
    vel_mag = np.sqrt(vx**2 + vy**2 + vz**2)
    return vel_mag


def compute_acceleration(dlio):
    ax = dlio["wx"].values * 0  # falls keine accelerometer-Daten: Dummy
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
