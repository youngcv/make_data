import xml.etree.ElementTree as ET

import numpy as np

from object_tools.sampling import sample_shuffled_axes, shuffled_configs
from object_tools.urdf import (
    ensure_standard_link_structure,
    set_visual_color,
    update_box_geometry,
)


def build_lighter_configs(body_size_ranges, cap_height_range, num_samples):
    body_axes = sample_shuffled_axes(body_size_ranges, num_samples)
    cap_heights = sample_shuffled_axes([cap_height_range], num_samples)[0]
    configs = [
        (
            tuple(samples[index] for samples in body_axes),
            cap_heights[index],
        )
        for index in range(num_samples)
    ]
    return shuffled_configs(configs)


def generate_lighter(
    input_path,
    output_path,
    config,
    joint_limits=None,
):
    body_size, cap_height = config
    body_x, body_y, body_z = body_size
    cap_size = (body_x, body_y, cap_height)

    tree = ET.parse(input_path)
    root = tree.getroot()
    ensure_standard_link_structure(root)

    link_0 = root.find(".//link[@name='link_0']")
    link_1 = root.find(".//link[@name='link_1']")
    joint_1 = root.find(".//joint[@name='joint_1']")
    if link_0 is None or link_1 is None or joint_1 is None:
        raise ValueError("Lighter URDF must contain link_0, link_1, and joint_1")

    update_box_geometry(link_0, body_size)
    cap_origin = np.array([0.0, 0.5 * body_y, 0.5 * cap_height])
    update_box_geometry(
        link_1,
        cap_size,
        " ".join(str(value) for value in cap_origin),
    )
    set_visual_color(link_1, "red", "1 0 0 1")

    joint_origin = np.array([0.0, -0.5 * body_y, 0.5 * body_z])
    origin = joint_1.find("origin")
    if origin is None:
        origin = ET.SubElement(joint_1, "origin")
    origin.set("xyz", " ".join(str(value) for value in joint_origin))
    origin.set("rpy", origin.get("rpy", "0 0 0"))

    limit = joint_1.find("limit")
    if limit is None:
        raise ValueError("Lighter joint_1 must contain a limit")

    if joint_limits is None:
        lower = float(limit.get("lower", 0.0))
        upper = float(limit.get("upper", 0.0))
    else:
        lower, upper = joint_limits
        limit.set("lower", f"{lower:.6f}")
        limit.set("upper", f"{upper:.6f}")
    if lower > upper:
        raise ValueError(f"joint lower {lower:.6f} is greater than upper {upper:.6f}")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)

    # Preserve the historical lbx.json ordering: link_1 first, then link_0.
    return (*cap_size, *body_size)
