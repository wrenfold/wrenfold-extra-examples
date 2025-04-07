"""
Generate the Snavely reprojection error via wrenfold.
"""

from pathlib import Path

from wrenfold import code_generation, sym
from wrenfold.geometry import Quaternion
from wrenfold.type_annotations import Vector2, Vector3, Vector9


def snavely_reprojection_error(camera: Vector9, point: Vector3, measured_xy: Vector2):
    """
    Symbolic implementation of the ceres `SnavelyReprojectionError`.

    See `simple_bundle_adjuster.cc` for the original implementation in C++.

    Args:
        camera: A 9-DOF vector with the camera paramters:
          - 3-DOF rotation vector.
          - 3-DOF camera translation.
          - 3 intrinsic parameters in order: [focal, l1, l2]
        point: Euclidean point position.
        measured_xy: Measured location in the image.
    """

    # Transform the point to camera frame:
    camera_R_world = Quaternion.from_rotation_vector(
        camera[0:3], epsilon=1.0e-16).to_rotation_matrix()

    p_camera = camera_R_world * point + camera[3:6]

    # Project into image using Snavely convention (negative z axis):
    xp = -p_camera[0] / p_camera[2]
    yp = -p_camera[1] / p_camera[2]

    # Apply the camera intrinsics:
    focal, l1, l2 = camera[6:]

    r2 = xp * xp + yp * yp
    distortion = 1 + r2 * (l1 + l2 * r2)

    residuals = (sym.vector(focal * distortion * xp, focal * distortion * yp) - measured_xy)
    return (
        code_generation.OutputArg(residuals, "residuals"),
        code_generation.OutputArg(
            sym.jacobian(residuals, camera), "residuals_D_camera", is_optional=True),
        code_generation.OutputArg(
            sym.jacobian(residuals, point), "residuals_D_point", is_optional=True),
    )


def main():
    generator = code_generation.CppGenerator()
    code = code_generation.generate_function(func=snavely_reprojection_error, generator=generator)
    code = generator.apply_preamble(code, namespace="gen")
    output_path = (Path(__file__).parent.absolute() / "generated" / "snavely_reprojection_error.h")
    code_generation.mkdir_and_write_file(code=code, path=output_path)


if __name__ == "__main__":
    main()
