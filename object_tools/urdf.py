import xml.etree.ElementTree as ET


def ensure_standard_link_structure(root):
    """Ensure base --joint_0(fixed)--> link_0 --joint_1(movable)--> link_1."""
    base_link = root.find("./link[@name='base']")
    if base_link is None:
        base_link = ET.Element("link", {"name": "base"})
        root.insert(0, base_link)

    movable_joint = root.find("./joint[@name='joint_1']")
    if movable_joint is None:
        movable_joint = root.find("./joint[@name='joint_0']")
    if movable_joint is not None:
        movable_joint.set("name", "joint_1")

    fixed_joint = root.find("./joint[@name='joint_0']")
    if fixed_joint is None:
        fixed_joint = ET.Element("joint", {"name": "joint_0", "type": "fixed"})
        root.insert(1, fixed_joint)

    fixed_joint.set("name", "joint_0")
    fixed_joint.set("type", "fixed")
    _set_origin(fixed_joint, "0 0 0", "0 0 0")
    _set_link_ref(fixed_joint, "parent", "base")
    _set_link_ref(fixed_joint, "child", "link_0")


def update_box_geometry(link_element, size_xyz, origin_xyz=None):
    size_str = " ".join(str(value) for value in size_xyz)
    for tag_name in ("visual", "collision"):
        node = link_element.find(tag_name)
        if node is None:
            continue

        geometry = node.find("geometry")
        if geometry is None:
            geometry = ET.SubElement(node, "geometry")
        for child in list(geometry):
            geometry.remove(child)
        ET.SubElement(geometry, "box", {"size": size_str})

        if origin_xyz is not None:
            origin = node.find("origin")
            if origin is None:
                origin = ET.SubElement(node, "origin")
            origin.set("xyz", origin_xyz)
            origin.set("rpy", origin.get("rpy", "0 0 0"))


def set_visual_color(link_element, material_name, rgba):
    for visual in link_element.findall("visual"):
        material = visual.find("material")
        if material is None:
            material = ET.SubElement(visual, "material")
        for child in list(material):
            material.remove(child)
        material.set("name", material_name)
        ET.SubElement(material, "color", {"rgba": rgba})


def _set_origin(parent, xyz, rpy):
    origin = parent.find("origin")
    if origin is None:
        origin = ET.SubElement(parent, "origin")
    origin.set("xyz", xyz)
    origin.set("rpy", rpy)


def _set_link_ref(joint, tag_name, link_name):
    node = joint.find(tag_name)
    if node is None:
        node = ET.SubElement(joint, tag_name)
    node.set("link", link_name)
