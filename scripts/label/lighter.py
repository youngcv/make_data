from object_tools.labeling import (
    MAT_BLACK,
    MAT_BLUE,
    MAT_GRAY,
    MAT_GREEN,
    MAT_RED,
    MAT_WHITE,
    make_label_processor,
    material_colors,
    run_label_batch,
    split_face_along_posy,
)


INPUT_DIR = "assets/objects/lighter"
OUTPUT_DIR = "assets/objects/lighter"
JOINT_VALUE_MODE = "upper"  # "lower" | "upper" | "mid" | "zero" | "custom"
CUSTOM_JOINT_VALUE = None
LINK0_POSY_WHITE_RATIO = 0.5
LINK0_NEGY_BLACK_RATIO = 0.5
LINK1_POSY_GREEN_RATIO = 0.3

MATERIAL_COLORS = {
    **material_colors(MAT_RED, MAT_BLUE, MAT_GREEN, MAT_GRAY),
    MAT_BLACK: (1.0, 0.75, 0.8),
    MAT_WHITE: (1.0, 0.75, 0.8),
}


def select_material(link_name, face_label, _world_normal):
    if link_name == "link_0":
        if face_label == "posz":
            return MAT_GRAY
        if face_label == "posy":
            return split_face_along_posy(MAT_RED, MAT_WHITE, LINK0_POSY_WHITE_RATIO)
        if face_label == "negy":
            return split_face_along_posy(MAT_RED, MAT_BLACK, LINK0_NEGY_BLACK_RATIO)
        if face_label == "negz":
            return MAT_BLUE
        return MAT_RED
    if link_name == "link_1":
        if face_label == "posy":
            return split_face_along_posy(MAT_RED, MAT_GREEN, LINK1_POSY_GREEN_RATIO)
        if face_label == "negz":
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
