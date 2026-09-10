import xml.etree.ElementTree as ET

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


JOINT_TRAVEL = 0.05


def build_knife_configs(
    body_size_ranges,
    slider_size_ranges,
    edge_clearance_range,
    num_samples,
    joint_travel=JOINT_TRAVEL,
):
    body_axes = sample_shuffled_axes(body_size_ranges, num_samples)
    slider_axes = sample_shuffled_axes(slider_size_ranges, num_samples)
    clearance_mix = sample_shuffled_range((0.0, 1.0), num_samples)

    configs = []
    for index in range(num_samples):
        body_size = tuple(samples[index] for samples in body_axes)
        slider_size = tuple(samples[index] for samples in slider_axes)
        clearance = _resolve_clearance(
            body_size[2],
            slider_size[2],
            edge_clearance_range,
            clearance_mix[index],
            joint_travel,
        )
        configs.append((body_size, slider_size, clearance))
    return shuffled_configs(configs)


def generate_knife(
    input_path,
    output_path,
    config,
    joint_travel=JOINT_TRAVEL,
):
    body_size, slider_size, edge_clearance = config
    tree = ET.parse(input_path)
    root = tree.getroot()
    ensure_standard_link_structure(root)

    _, body_y, body_z = body_size
    _, slider_y, slider_z = slider_size
    slider_origin_y = 0.5 * (body_y + slider_y)
    lower = edge_clearance - 0.5 * (body_z - slider_z)
    if joint_travel <= 0.0:
        raise ValueError("joint travel must be greater than zero")
    upper = lower + joint_travel

    upper_edge_clearance = body_z - slider_z - edge_clearance - joint_travel
    if upper_edge_clearance < 0.0:
        raise ValueError(
            "Knife dimensions and edge clearance are incompatible with the "
            f"{joint_travel:.3f} joint travel."
        )

    link_0 = root.find(".//link[@name='link_0']")
    link_1 = root.find(".//link[@name='link_1']")
    joint_1 = root.find(".//joint[@name='joint_1']")
    if link_0 is None or link_1 is None or joint_1 is None:
        raise ValueError("Knife URDF must contain link_0, link_1, and joint_1")

    update_box_geometry(link_0, body_size)
    child_origin = f"0 {slider_origin_y:.6f} 0"
    update_box_geometry(link_1, slider_size, child_origin)
    set_visual_color(link_1, "red", "1 0 0 1")

    limit = joint_1.find("limit")
    if limit is None:
        limit = ET.SubElement(
            joint_1,
            "limit",
            {"effort": "100", "velocity": "1.0"},
        )
    limit.set("lower", f"{lower:.6f}")
    limit.set("upper", f"{upper:.6f}")

    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return (*body_size, *slider_size)


def _resolve_clearance(body_z, slider_z, clearance_range, mix, joint_travel):
    clearance_low, clearance_high = clearance_range
    if clearance_low > clearance_high:
        raise ValueError("edge clearance lower bound cannot exceed upper bound")

    free_length = body_z - slider_z
    max_clearance = min(0.5 * free_length, free_length - joint_travel)
    if clearance_low > max_clearance:
        raise ValueError(
            "Knife clearance range is incompatible with the sampled body/slider "
            f"lengths and {joint_travel:.3f} joint travel."
        )

    effective_high = min(clearance_high, max_clearance)
    return clearance_low + mix * (effective_high - clearance_low)
