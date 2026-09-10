import xml.etree.ElementTree as ET

import numpy as np

from object_tools.geometry import axis_angle_to_matrix
from object_tools.sampling import (
    sample_shuffled_axes,
    sample_shuffled_range,
    shuffled_configs,
)
from object_tools.urdf import (
    ensure_standard_link_structure,
    set_visual_color,
    update_box_geometry,
)


HINGE_AXIS = (-1.0, 0.0, 0.0)


def build_equal_link_configs(
    size_ranges,
    joint_lower_range,
    num_samples,
    min_z_filter_joint_offset=None,
):
    configs = []
    attempts = 0
    max_attempts = max(1000, num_samples * 100)

    while len(configs) < num_samples and attempts < max_attempts:
        remaining = num_samples - len(configs)
        sampled_axes = sample_shuffled_axes(size_ranges, remaining)
        lower_samples = sample_shuffled_range(joint_lower_range, remaining)
        attempts += remaining

        for idx in range(remaining):
            shared_size_xyz = tuple(axis_samples[idx] for axis_samples in sampled_axes)
            joint_lower = lower_samples[idx]
            if min_z_filter_joint_offset is not None:
                joint_value = joint_lower + min_z_filter_joint_offset
                if link_1_bottom_is_below_link_0(shared_size_xyz, joint_value):
                    continue
            configs.append((shared_size_xyz, joint_lower))

    if len(configs) < num_samples:
        raise ValueError(
            "Could not sample enough hinged-pair objects that satisfy the "
            "link_1/link_0 minimum-z filter. Relax the size ranges, joint "
            "lower range, or filter joint offset."
        )
    return shuffled_configs(configs)


def generate_equal_hinged_pair(
    input_path,
    output_path,
    config,
    joint_upper_value=None,
):
    shared_size_xyz, joint_lower_value = config
    tree = ET.parse(input_path)
    root = tree.getroot()
    ensure_standard_link_structure(root)

    link_0 = root.find(".//link[@name='link_0']")
    link_1 = root.find(".//link[@name='link_1']")
    if link_0 is None or link_1 is None:
        raise ValueError("URDF must contain link_0 and link_1")

    update_box_geometry(link_0, shared_size_xyz)
    _, size_y, size_z = shared_size_xyz
    child_origin = np.array([0.0, 0.5 * size_y, 0.5 * size_z])
    child_origin_str = " ".join(str(value) for value in child_origin)
    update_box_geometry(link_1, shared_size_xyz, child_origin_str)
    set_visual_color(link_1, "red", "1 0 0 1")

    joint_origin = np.array([0.0, -0.5 * size_y, 0.5 * size_z])
    joint_1 = root.find(".//joint[@name='joint_1']")
    if joint_1 is None:
        raise ValueError("URDF must contain movable joint_1")

    origin = joint_1.find("origin")
    if origin is None:
        origin = ET.SubElement(joint_1, "origin")
    origin.set("xyz", " ".join(str(value) for value in joint_origin))
    origin.set("rpy", origin.get("rpy", "0 0 0"))

    limit = joint_1.find("limit")
    if limit is None:
        raise ValueError("joint_1 must contain a limit")

    upper = (
        float(limit.get("upper", 0.0))
        if joint_upper_value is None
        else float(joint_upper_value)
    )
    lower = float(joint_lower_value)
    if lower > upper:
        raise ValueError(f"joint lower {lower:.6f} is greater than upper {upper:.6f}")

    limit.set("lower", f"{lower:.6f}")
    limit.set("upper", f"{upper:.6f}")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return (*shared_size_xyz, *shared_size_xyz)


def link_1_bottom_is_below_link_0(shared_size_xyz, joint_value, tolerance=1e-9):
    """Compare the -z face vertices of link_1 and link_0 at one joint pose."""
    link_1_min_z = _link_1_bottom_face_min_z(shared_size_xyz, joint_value)
    link_0_min_z = -0.5 * shared_size_xyz[2]
    return link_1_min_z < link_0_min_z - tolerance


def _link_1_bottom_face_min_z(shared_size_xyz, joint_value):
    size_x, size_y, size_z = shared_size_xyz
    half_x = 0.5 * size_x
    half_y = 0.5 * size_y
    half_z = 0.5 * size_z
    visual_origin = np.array([0.0, half_y, half_z])
    joint_origin = np.array([0.0, -half_y, half_z])
    rotation = axis_angle_to_matrix(HINGE_AXIS, joint_value)

    bottom_face_vertices = np.array(
        [
            [-half_x, -half_y, -half_z],
            [half_x, -half_y, -half_z],
            [half_x, half_y, -half_z],
            [-half_x, half_y, -half_z],
        ]
    )
    world_vertices = joint_origin + (rotation @ (bottom_face_vertices + visual_origin).T).T
    return float(np.min(world_vertices[:, 2]))
