from object_tools.labeling import (
    MAT_BLUE,
    MAT_GRAY,
    MAT_GREEN,
    MAT_RED,
    make_label_processor,
    material_colors,
    run_label_batch,
)


INPUT_DIR = "assets/objects/knife"
OUTPUT_DIR = "assets/objects/knife"
JOINT_VALUE_MODE = "lower"  # "lower" | "upper" | "mid" | "zero" | "custom"
CUSTOM_JOINT_VALUE = None

MATERIAL_COLORS = material_colors(MAT_RED, MAT_BLUE, MAT_GREEN, MAT_GRAY)


def select_material(link_name, face_label, _world_normal):
    if link_name == "link_0":
        return MAT_BLUE if face_label == "negy" else MAT_RED
    if link_name == "link_1":
        if face_label == "posy":
            return MAT_GREEN
        if face_label == "negy":
            return MAT_GRAY
        return MAT_RED
    return MAT_GRAY


process_urdf = make_label_processor(
    select_material,
    MATERIAL_COLORS,
    JOINT_VALUE_MODE,
    CUSTOM_JOINT_VALUE,
)


def main(data_root=None):
    run_label_batch(
        data_root or INPUT_DIR,
        data_root or OUTPUT_DIR,
        process_urdf,
    )


if __name__ == "__main__":
    main()
