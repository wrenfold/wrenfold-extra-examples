"""
Generate a function that interpolates between two Sophus::SE3 objects (in the tangent space),
and produces derivatives with respect to both poses.

This is intended as an example of customizing code-generation to work with Sophus types. You could
likely adapt this to work with your own SE(3) pose type.
"""

from pathlib import Path

from wrenfold import ast, code_generation, sym, type_info
from wrenfold.type_annotations import FloatScalar

from ..pose3 import Pose3, Rot3


class SophusCppGenerator(code_generation.CppGenerator):
    """
    Customize the code-generation to work with Sophus types.
    """

    def format_get_field(self, element: ast.GetField) -> str:
        """
        Customize member access to work with SE3 and SO3.
        """
        if element.struct_type.python_type == Rot3:
            return (f"{self.format(element.arg)}.unit_quaternion().{element.field_name}()")
        elif element.struct_type.python_type == Pose3:
            if element.field_name == "rotation":
                return f"{self.format(element.arg)}.so3()"
            else:
                return f"{self.format(element.arg)}.translation()"

        return self.super_format(element)

    def format_construct_custom_type(self, element: ast.ConstructCustomType) -> str:
        """
        Customize the construction of Sophus::SO3. We need to construct a quaternion first and pass
        that to the constructor of SO3.

        SE3 requires no customization, the default formatter can handle that.
        """
        if element.type.python_type == Rot3:
            # Eigen expects things in order [w, x, y, z].
            fields_order = ["w", "x", "y", "z"]
            w, x, y, z = [self.format(element.get_field_value(name)) for name in fields_order]
            return f"{self.format(element.type)}(Eigen::Quaternion<Scalar>({w}, {x}, {y}, {z}))"

        return self.super_format(element)

    def format_construct_matrix(self, element: ast.ConstructMatrix) -> str:
        """
        We need to tell the code-generator how we want matrices constructed.
        """
        if element.type.shape == (3, 1):
            x, y, z = element.args
            return f"Eigen::Vector3<Scalar>({self.format(x)}, {self.format(y)}, {self.format(z)})"

        return self.super_format(element)

    def format_custom_type(self, element: type_info.CustomType) -> str:
        """Place types into the `Sophus` namespace."""
        if element.python_type == Pose3:
            return f"Sophus::SE3<Scalar>"
        elif element.python_type == Rot3:
            return f"Sophus::SO3<Scalar>"
        return self.super_format(element)


def pose3_interpolate(pose_a: Pose3, pose_b: Pose3, alpha: FloatScalar):
    """
    Interpolate between two poses on SE(3). `alpha` is the the interpolation fraction between
    [0, 1]. We perform the operation:

    result = pose_a * exp(log(pose_a^-1 * pose_b) * alpha)

    And compute the tangent-space derivatives of the result with respect to both poses.
    """
    pose_interpolated = pose_a.retract(pose_a.local_coordinates(pose_b) * alpha)

    # Compute derivatives wrt the two poses:
    tangent_D_pose = pose_interpolated.right_local_coordinates_derivative()
    D_a = (
        tangent_D_pose * sym.jacobian(pose_interpolated.to_vector(), pose_a.to_vector()) *
        pose_a.right_retract_derivative())
    D_b = (
        tangent_D_pose * sym.jacobian(pose_interpolated.to_vector(), pose_b.to_vector()) *
        pose_b.right_retract_derivative())

    # We could get the derivative wrt alpha too, if we wanted to:
    #   D_alpha = tangent_D_pose * sym.jacobian(pose_interpolated.to_vector(), [alpha])

    return [
        code_generation.ReturnValue(pose_interpolated),
        code_generation.OutputArg(D_a, name="D_a", is_optional=True),
        code_generation.OutputArg(D_b, name="D_b", is_optional=True),
    ]


def pose3_interpolate_first_order(pose_a: Pose3, pose_b: Pose3, alpha: FloatScalar):
    """
    Computes the same function as `pose3_interpolate`. However, we compute the tangent-space
    jacobians by applying a first order expansion of the retraction and local-coords operators,
    then evaluating around zero.

    This allows for further automatic simplifications while producing numerically the same result.
    """
    perturb_a = sym.vector(*sym.unique_symbols(count=6, real=True))
    perturb_b = sym.vector(*sym.unique_symbols(count=6, real=True))
    all_perturbations = sym.vstack([perturb_a, perturb_b])

    # Perturb the input poses w/ the first order
    pose_a = pose_a.retract_first_order(perturb_a)
    pose_b = pose_b.retract_first_order(perturb_b)

    pose_interpolated_perturbed = pose_a.retract(pose_a.local_coordinates(pose_b) * alpha)

    # Compute the output by setting the perturbation to zero:
    pose_interpolated = Pose3.from_vector(pose_interpolated_perturbed.to_vector().subs([
        (v, 0) for v in all_perturbations
    ]))

    D_a = sym.jacobian(
        pose_interpolated.local_coordinates_first_order(pose_interpolated_perturbed),
        perturb_a,
    ).subs([(v, 0) for v in all_perturbations])

    D_b = sym.jacobian(
        pose_interpolated.local_coordinates_first_order(pose_interpolated_perturbed),
        perturb_b,
    ).subs([(v, 0) for v in all_perturbations])

    return [
        code_generation.ReturnValue(pose_interpolated),
        code_generation.OutputArg(D_a, name="D_a", is_optional=True),
        code_generation.OutputArg(D_b, name="D_b", is_optional=True),
    ]


def main():
    code = "\n\n".join([
        code_generation.generate_function(
            func=pose3_interpolate,
            generator=SophusCppGenerator(),
        ),
        code_generation.generate_function(
            func=pose3_interpolate_first_order,
            generator=SophusCppGenerator(),
        ),
    ])
    code = code_generation.CppGenerator.apply_preamble(
        code, namespace="gen", imports="#include <sophus/se3.hpp>")
    output_path = Path(__file__).parent.absolute() / "generated" / "pose3_interpolate.h"
    code_generation.mkdir_and_write_file(code=code, path=output_path)


if __name__ == "__main__":
    main()
