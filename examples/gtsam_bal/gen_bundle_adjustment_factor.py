"""
Generate a GTSAM reprojection error for use with BAL datasets.
"""

import dataclasses
from pathlib import Path
import typing as T

from wrenfold import sym
from wrenfold import code_generation
from wrenfold.geometry import Quaternion, left_jacobian_of_so3
from wrenfold.type_annotations import FloatScalar, Vector4, Vector3, Vector2
from wrenfold import ast
from wrenfold import type_info


@dataclasses.dataclass
class GtsamQuaternion:
    """
    A rotation in three dimensions, represented by a quaternion. In actuality,
    this is just an Eigen quaternion, ordered [x, y, z, w].
    """

    x: FloatScalar
    y: FloatScalar
    z: FloatScalar
    w: FloatScalar

    def to_quaternion(self) -> Quaternion:
        """Convert to the wrenfold quaternion type, so we can more easily manipulate symbolically."""
        return Quaternion(w=self.w, x=self.x, y=self.y, z=self.z)

    @staticmethod
    def from_quaternion(q: Quaternion) -> "GtsamQuaternion":
        """Construct from wrenfold quaternion."""
        return GtsamQuaternion(x=q.x, y=q.y, z=q.z, w=q.w)

    def rotation_matrix(self) -> sym.MatrixExpr:
        """Convert to 3x3 rotation matrix."""
        return self.to_quaternion().to_rotation_matrix()

    def to_flat_list(self) -> T.List[sym.Expr]:
        return [self.x, self.y, self.z, self.w]


@dataclasses.dataclass
class GtsamPose3:
    """
    Custom type that maps to GTSAM Pose3 in generated code.

    Because gtsam::Pose3 uses SE(3), we need to define the derivative of the retraction
    operation (implemented in `right_retract_derivative` below).
    """

    rotation: GtsamQuaternion
    translation: Vector3

    def to_flat_list(self) -> T.List[sym.Expr]:
        """
        Flatten to a single list of expressions.
        """
        return self.rotation.to_flat_list() + self.translation.to_flat_list()

    def compose(self, other: "GtsamPose3") -> "GtsamPose3":
        """
        The product of two poses.
        """
        return GtsamPose3(
            rotation=GtsamQuaternion.from_quaternion(self.rotation.to_quaternion() *
                                                     other.rotation.to_quaternion()),
            translation=self.translation + self.rotation.rotation_matrix() * other.translation,
        )

    def right_retract_derivative(self) -> sym.MatrixExpr:
        """
        The 7x6 derivative of the 7 (4 quaternion + 3 translation) pose elements with respect
        to the right-tangent space.

        """
        # First create a symbolic pose `X`. We will replace this variables with the contents of
        # `self` after doing some manipulations.
        R_symbolic = Quaternion.with_name("q")
        t_symbolic = sym.vector(*sym.symbols("t_x, t_y, t_z"))
        X = GtsamPose3(
            rotation=GtsamQuaternion.from_quaternion(R_symbolic),
            translation=t_symbolic,
        )

        # Create a tangent-space perturbation, and perturb pose `X`
        dw = sym.vector(*sym.symbols("dw_x, dw_y, dw_z"))
        dt = sym.vector(*sym.symbols("dt_x, dt_y, dt_z"))
        dR = Quaternion.from_rotation_vector(dw, epsilon=0)
        X_perturbed = X.compose(
            GtsamPose3(
                rotation=GtsamQuaternion.from_quaternion(dR),
                translation=left_jacobian_of_so3(dw, epsilon=0) * dt,
            ))

        # Compute the jacobian wrt the tangent space perturbation:
        J = sym.jacobian(X_perturbed.to_flat_list(), dw.to_flat_list() + dt.to_flat_list())
        assert J.shape == (7, 6)

        # Evaluate about perturbation = 0
        J = J.subs([(var, 0) for var in dw] + [(var, 0) for var in dt])

        # Substitute the values of self:
        return J.subs(list(zip(X.to_flat_list(), self.to_flat_list())))

    def inverse(self) -> "GtsamPose3":
        """
        If `A` is the 4x4 homogeneous transform corresponding to `self`, we compute the pose that
        corresponds to the matrix `A^-1` such that A * A^-1 --> Identity.
        """
        q_inv: Quaternion = self.rotation.to_quaternion().conjugate()
        return GtsamPose3(
            rotation=GtsamQuaternion.from_quaternion(q_inv),
            translation=q_inv.to_rotation_matrix() * -self.translation,
        )


@dataclasses.dataclass
class Cal3Bundler:
    """
    Corresponds to type gtsam::Cal3Bundler (a 3 DOF Snavely camera model).
    """

    f: FloatScalar
    k1: FloatScalar
    k2: FloatScalar

    def to_flat_list(self) -> T.List[sym.Expr]:
        return [self.f, self.k1, self.k2]

    def project(self, p: Vector3) -> Vector2:
        """
        Project and apply intrinsic model.

        We could improve this by adding a conditional here that handles p[2] <= 0.
        """
        xp = p[0] / p[2]
        yp = p[1] / p[2]

        r2 = xp * xp + yp * yp
        distortion = 1 + r2 * (self.k1 + self.k2 * r2)
        return sym.vector(self.f * distortion * xp, self.f * distortion * yp)


@dataclasses.dataclass
class SfmCamera:
    """
    Corresponds to type gtsam::SfmCamera.
    """

    pose: GtsamPose3
    calibration: Cal3Bundler

    def to_flat_list(self) -> T.List[sym.Expr]:
        return self.pose.to_flat_list() + self.calibration.to_flat_list()


class CustomCppGenerator(code_generation.CppGenerator):
    """
    Customize the code-generation to work with GTSAM types.
    """

    def format_get_field(self, element: ast.GetField) -> str:
        """
        Customize access to `gtsam::Pose3` and `Cal3Bundler`.
        """
        if (element.struct_type.python_type == GtsamPose3 and element.field_name == "rotation"):
            # Retrieve Rot3, and then the Eigen Quaternion:
            return f"{self.format(element.arg)}.rotation().toQuaternion()"
        elif (element.struct_type.python_type == Cal3Bundler and element.field_name == "f"):
            # fx() == fy() for the Cal3Bundler type:
            return f"{self.format(element.arg)}.fx()"

        return f"{self.format(element.arg)}.{element.field_name}()"

    def format_custom_type(self, element: type_info.CustomType) -> str:
        """Place types into the `gtsam` namespace."""
        if element.python_type in [GtsamPose3, GtsamQuaternion, Cal3Bundler, SfmCamera]:
            return f"gtsam::{element.name}"
        return self.super_format(element)


def bundle_adjustment_factor(camera: SfmCamera, p_world: Vector3):
    """
    Transforms Euclidean point `p_world` into the frame of `camera`, and then projects it
    using a simplified intrinsic model with three parameters [f, k1, k2].
    """
    # Transform the point from world to camera:
    world_T_camera = camera.pose
    p_cam = world_T_camera.rotation.rotation_matrix().T * (p_world - world_T_camera.translation)

    # Project and compute projection error
    p_image = camera.calibration.project(p_cam)

    # Compute jacobian wrt the flattened pose and intrinsic model:
    p_image_D_camera = sym.jacobian(p_image, camera.to_flat_list())
    assert p_image_D_camera.shape == (2, 10)

    # Convert it to be with respect to the right-tangent space of the camera pose:
    p_image_D_camera_tangent = p_image_D_camera * sym.diag(
        [camera.pose.right_retract_derivative(), sym.eye(3)])
    assert p_image_D_camera_tangent.shape == (2, 9)

    # Compute jacobian wrt the point:
    p_image_D_point = sym.jacobian(p_image, p_world)

    return (
        code_generation.OutputArg(p_image, "p_image"),
        code_generation.OutputArg(p_image_D_camera_tangent, "p_image_D_camera", is_optional=True),
        code_generation.OutputArg(p_image_D_point, "p_image_D_point", is_optional=True),
    )


def main():
    code = code_generation.generate_function(
        func=bundle_adjustment_factor, generator=CustomCppGenerator())
    code = code_generation.CppGenerator.apply_preamble(code, namespace="gen")
    output_path = (Path(__file__).parent.absolute() / "generated" / "bundle_adjustment_factor.h")
    code_generation.mkdir_and_write_file(code=code, path=output_path)


if __name__ == "__main__":
    main()
