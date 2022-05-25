import numpy as np

from kinematics import *


def axis_uncertainty(T):
    projection = np.diagonal(T)
    uncertainty = np.arccos(projection[:-1])
    return uncertainty


def converter(uncertainty):
    roll, pitch, yaw = uncertainty
    dc = DefaultConfigurations()
    hole_top_uncertainty_T = homo([roll, pitch, yaw, 0.628, 0, dc.hole_height_wrt_franka])
    ax_uncertainty = axis_uncertainty(hole_top_uncertainty_T)

    return ax_uncertainty
    # ee_T_based_on_franka, ee_config = fixed_hole_top_to_franka_ee(hole_top_uncertainty_T, dc)


def main():
    grid = np.linspace(-0.05, 0.05, 11)
    mesh = np.vstack(np.meshgrid(grid, grid, grid)).reshape(3, -1).T
    ax_uncertainty = np.zeros_like(mesh)
    for i, uncertainty in enumerate(mesh):
        ax_uncertainty[i, :] = converter(uncertainty)
    print(ax_uncertainty)


if __name__ == "__main__":
    main()
