import inspect
import os
import xml.etree.ElementTree as ET

import numpy as np

from object_tools.geometry import (
    axis_angle_to_matrix,
    resolve_joint_value,
    rpy_to_matrix,
)


MAT_RED = "red"
MAT_BLUE = "blue"
MAT_GREEN = "green"
MAT_GRAY = "gray"
MAT_BLACK = "black"
MAT_WHITE = "white"

MATERIAL_PALETTE = {
    MAT_RED: (1.0, 0.0, 0.0),
    MAT_BLUE: (0.0, 0.0, 1.0),
    MAT_GREEN: (0.0, 1.0, 0.0),
    MAT_GRAY: (0.7, 0.7, 0.7),
}


def material_colors(*material_names):
    return {name: MATERIAL_PALETTE[name] for name in material_names}


def run_label_batch(input_dir, output_dir, process_one):
    """Process every numbered object directory with one labeling function."""
    if not os.path.exists(input_dir):
        print(f"Error: Input directory '{input_dir}' does not exist.")
        return

    os.makedirs(output_dir, exist_ok=True)
    object_ids = sorted(
        object_id
        for object_id in os.listdir(input_dir)
        if object_id.isdigit()
        and os.path.isdir(os.path.join(input_dir, object_id))
    )
    print(f"Found {len(object_ids)} items to process.")

    for object_id in object_ids:
        urdf_path = os.path.join(input_dir, object_id, "mobility.urdf")
        if not os.path.exists(urdf_path):
            print(f"[Skip] {object_id}: mobility.urdf not found.")
            continue

        obj_path = os.path.join(output_dir, object_id, "labeled.obj")
        mtl_path = os.path.join(output_dir, object_id, "labeled.mtl")
        try:
            process_one(urdf_path, obj_path, mtl_path)
            print(f"[Done] Processed {object_id}")
        except Exception as exc:
            print(f"[Error] Failed to process {object_id}: {exc}")


def make_transform(xyz, rpy):
    transform = np.eye(4)
    transform[:3, :3] = rpy_to_matrix(rpy)
    transform[:3, 3] = xyz
    return transform


def parse_origin(elem):
    if elem is None:
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    xyz_str = elem.attrib.get("xyz", "0 0 0")
    rpy_str = elem.attrib.get("rpy", "0 0 0")
    xyz = tuple(map(float, xyz_str.split()))
    rpy = tuple(map(float, rpy_str.split()))
    return xyz, rpy


def parse_origin_matrix(elem):
    xyz, rpy = parse_origin(elem)
    return make_transform(xyz, rpy)


def get_joint_value(joint_node, mode, custom_value):
    if custom_value is not None:
        return float(custom_value)

    limit_node = joint_node.find("limit")
    lower = upper = 0.0
    if limit_node is not None:
        lower = float(limit_node.get("lower", 0.0))
        upper = float(limit_node.get("upper", 0.0))

    if mode == "custom":
        raise ValueError("JOINT_VALUE_MODE='custom' 时必须设置 CUSTOM_JOINT_VALUE")
    return resolve_joint_value(mode, lower, upper)


def get_joint_transform(joint_node, mode, custom_value):
    if joint_node is None:
        return np.eye(4)

    static_transform = parse_origin_matrix(joint_node.find("origin"))
    joint_type = joint_node.get("type", "fixed")
    if joint_type == "fixed":
        return static_transform

    joint_value = get_joint_value(joint_node, mode, custom_value)
    axis_node = joint_node.find("axis")
    axis = np.array([1.0, 0.0, 0.0], dtype=float)
    if axis_node is not None:
        axis = np.array([float(x) for x in axis_node.get("xyz", "1 0 0").split()])
        axis_norm = np.linalg.norm(axis)
        if axis_norm > 1e-8:
            axis /= axis_norm

    motion_transform = np.eye(4)
    if joint_type == "prismatic":
        motion_transform[:3, 3] = axis * joint_value
    elif joint_type in ["revolute", "continuous"] and np.linalg.norm(axis) > 1e-8:
        motion_transform[:3, :3] = axis_angle_to_matrix(axis, joint_value)

    return static_transform @ motion_transform


def make_box(extents):
    x, y, z = 0.5 * np.array(extents, dtype=float)
    vertices = np.array(
        [
            [-x, -y, -z],
            [x, -y, -z],
            [x, y, -z],
            [-x, y, -z],
            [-x, -y, z],
            [x, -y, z],
            [x, y, z],
            [-x, y, z],
        ]
    )
    faces = [
        ((0, 3, 2, 1), "negz", np.array([0.0, 0.0, -1.0])),
        ((4, 5, 6, 7), "posz", np.array([0.0, 0.0, 1.0])),
        ((0, 1, 5, 4), "negy", np.array([0.0, -1.0, 0.0])),
        ((2, 3, 7, 6), "posy", np.array([0.0, 1.0, 0.0])),
        ((1, 2, 6, 5), "posx", np.array([1.0, 0.0, 0.0])),
        ((0, 4, 7, 3), "negx", np.array([-1.0, 0.0, 0.0])),
    ]
    return vertices, faces


def apply_transform(vertices, transform):
    ones = np.ones((len(vertices), 1))
    vertices_h = np.hstack([vertices, ones])
    return (transform @ vertices_h.T).T[:, :3]


def load_box_components(root):
    components = []
    for link in root.findall("link"):
        link_name = link.attrib["name"]
        visuals = link.findall("visual")
        for visual_idx, visual in enumerate(visuals):
            box = visual.find("geometry/box")
            if box is None:
                continue

            xyz, rpy = parse_origin(visual.find("origin"))
            size = tuple(map(float, box.attrib["size"].split()))
            components.append(
                {
                    "link_name": link_name,
                    "component_name": visual.attrib.get(
                        "name",
                        f"{link_name}_{visual_idx}",
                    ),
                    "size": size,
                    "visual_transform": make_transform(xyz, rpy),
                }
            )
    return components


def build_world_transforms(root, link_names, joint_value_mode, custom_joint_value):
    joints = root.findall("joint")
    if not joints:
        return {name: np.eye(4) for name in link_names}

    parent_links = []
    child_links = set()
    for joint in joints:
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        parent_links.append(parent)
        child_links.add(child)

    transforms = {}
    for parent in parent_links:
        if parent not in child_links:
            transforms[parent] = np.eye(4)

    pending_joints = list(joints)
    while pending_joints:
        next_pending = []
        progress = False
        for joint in pending_joints:
            parent = joint.find("parent").attrib["link"]
            child = joint.find("child").attrib["link"]
            if parent not in transforms:
                next_pending.append(joint)
                continue

            transforms[child] = transforms[parent] @ get_joint_transform(
                joint,
                joint_value_mode,
                custom_joint_value,
            )
            progress = True

        if not progress:
            break
        pending_joints = next_pending

    for link_name in link_names:
        transforms.setdefault(link_name, np.eye(4))

    return transforms


def build_mesh_records(components, world_transforms, material_selector):
    all_vertices = []
    face_records = []
    vertex_offset = 1
    selector_arity = len(inspect.signature(material_selector).parameters)

    for component_data in components:
        link_name = component_data["link_name"]
        component_name = component_data["component_name"]
        if link_name not in world_transforms:
            continue

        local_vertices, faces = make_box(component_data["size"])
        total_transform = (
            world_transforms[link_name] @ component_data["visual_transform"]
        )
        world_vertices = apply_transform(local_vertices, total_transform)
        world_rotation = total_transform[:3, :3]
        extra_vertices = []

        for indices, face_label, local_normal in faces:
            world_normal = world_rotation @ local_normal
            normal_norm = np.linalg.norm(world_normal)
            if normal_norm > 1e-8:
                world_normal /= normal_norm

            if selector_arity >= 4:
                material_name = material_selector(
                    link_name,
                    component_name,
                    face_label,
                    world_normal,
                )
            else:
                material_name = material_selector(link_name, face_label, world_normal)
            if isinstance(material_name, dict) and material_name.get("type") == "split_posy":
                face_vertices = np.array([world_vertices[idx] for idx in indices])
                split_t = material_name.get("split_t", 0.5)
                mid_left = (1.0 - split_t) * face_vertices[0] + split_t * face_vertices[3]
                mid_right = (1.0 - split_t) * face_vertices[1] + split_t * face_vertices[2]

                mid_left_idx = len(local_vertices) + len(extra_vertices)
                extra_vertices.append(mid_left)
                mid_right_idx = len(local_vertices) + len(extra_vertices)
                extra_vertices.append(mid_right)

                face_records.append(
                    (
                        material_name["negative_material"],
                        np.array([indices[0], indices[1], mid_right_idx, mid_left_idx]) + vertex_offset,
                    )
                )
                face_records.append(
                    (
                        material_name["positive_material"],
                        np.array([mid_left_idx, mid_right_idx, indices[2], indices[3]]) + vertex_offset,
                    )
                )
            else:
                face_records.append((material_name, np.array(indices) + vertex_offset))

        if extra_vertices:
            world_vertices = np.vstack([world_vertices, np.array(extra_vertices)])

        all_vertices.append(world_vertices)
        vertex_offset += len(world_vertices)

    return all_vertices, face_records


def split_face_along_posy(negative_material, positive_material, positive_ratio=0.5):
    if positive_ratio <= 0.0:
        return negative_material
    if positive_ratio >= 1.0:
        return positive_material

    return {
        "type": "split_posy",
        "negative_material": negative_material,
        "positive_material": positive_material,
        "split_t": 1.0 - positive_ratio,
    }


def hinged_pair_material(link_name, face_label, _world_normal):
    if link_name == "link_0":
        return MAT_BLUE if face_label == "negz" else MAT_RED
    if link_name == "link_1" and face_label == "posz":
        return split_face_along_posy(MAT_RED, MAT_GREEN)
    return MAT_RED


def make_label_processor(
    material_selector,
    material_colors,
    joint_value_mode,
    custom_joint_value=None,
):
    def process(urdf_path, obj_path, mtl_path):
        process_labeled_urdf(
            urdf_path,
            obj_path,
            mtl_path,
            material_selector=material_selector,
            material_colors=material_colors,
            joint_value_mode=joint_value_mode,
            custom_joint_value=custom_joint_value,
        )

    return process


def run_hinged_pair_label_batch(
    input_dir,
    output_dir,
    joint_value_mode="lower",
    custom_joint_value=None,
):
    process = make_label_processor(
        hinged_pair_material,
        material_colors(MAT_RED, MAT_BLUE, MAT_GREEN),
        joint_value_mode,
        custom_joint_value,
    )
    run_label_batch(input_dir, output_dir, process)


def write_mtl(mtl_path, material_colors):
    with open(mtl_path, "w") as handle:
        for material_name, color in material_colors.items():
            r, g, b = color
            handle.write(f"newmtl {material_name}\n")
            handle.write(f"Kd {r} {g} {b}\n\n")


def write_obj(obj_path, mtl_path, all_vertices, face_records):
    mtl_filename = os.path.basename(mtl_path)
    with open(obj_path, "w") as handle:
        handle.write(f"mtllib {mtl_filename}\n")

        for vertex_array in all_vertices:
            for vertex in vertex_array:
                handle.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")

        current_material = None
        for material_name, indices in face_records:
            if material_name != current_material:
                handle.write(f"usemtl {material_name}\n")
                current_material = material_name
            handle.write(f"f {indices[0]} {indices[1]} {indices[2]} {indices[3]}\n")


def process_labeled_urdf(
    urdf_path,
    obj_path,
    mtl_path,
    material_selector,
    material_colors,
    joint_value_mode="lower",
    custom_joint_value=None,
):
    os.makedirs(os.path.dirname(obj_path), exist_ok=True)
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    components = load_box_components(root)
    world_transforms = build_world_transforms(
        root,
        {component["link_name"] for component in components},
        joint_value_mode,
        custom_joint_value,
    )
    all_vertices, face_records = build_mesh_records(
        components,
        world_transforms,
        material_selector,
    )

    write_mtl(mtl_path, material_colors)
    write_obj(obj_path, mtl_path, all_vertices, face_records)
