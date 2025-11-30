import numpy as np
from scipy.stats import linregress, gaussian_kde

# ---------------------------
# Quaternion → Yaw
# ---------------------------
def quaternion_to_yaw(qx, qy, qz, qw):
    return np.arctan2(
        2*(qw*qz + qx*qy),
        1 - 2*(qy*qy + qz*qz)
    )

# ---------------------------
# Yaw Error
# ---------------------------
def compute_yaw_error(dlio, lio_interp):
    yaw_lio = quaternion_to_yaw(
        lio_interp["qx"],
        lio_interp["qy"],
        lio_interp["qz"],
        lio_interp["qw"]
    )
    yaw_dlio = quaternion_to_yaw(
        dlio["qx"].values,
        dlio["qy"].values,
        dlio["qz"].values,
        dlio["qw"].values
    )
    return np.unwrap(yaw_dlio - yaw_lio)

# ---------------------------
# Regression für Error vs. Yaw
# ---------------------------
def compute_regression(yaw_err, err_xy):
    return linregress(yaw_err, err_xy)

# ---------------------------
# Yaw Derivative
# ---------------------------
def compute_yaw_derivative(t, yaw_err):
    dt = np.gradient(t)
    return np.gradient(yaw_err) / dt

# ---------------------------
# KDE für Violin Plot
# ---------------------------
def compute_kde(err_norm):
    kde = gaussian_kde(err_norm)
    y = np.linspace(err_norm.min(), err_norm.max(), 300)
    density = kde(y)
    density = density / density.max() * 0.4
    return y, density
