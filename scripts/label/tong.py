from object_tools.labeling import run_hinged_pair_label_batch


INPUT_DIR = "assets/objects/tong"
OUTPUT_DIR = "assets/objects/tong"
JOINT_VALUE_MODE = "lower"  # "lower" | "upper" | "mid" | "zero" | "custom"
CUSTOM_JOINT_VALUE = None


def main(data_root=None):
    run_hinged_pair_label_batch(
        data_root or INPUT_DIR,
        data_root or OUTPUT_DIR,
        JOINT_VALUE_MODE,
        CUSTOM_JOINT_VALUE,
    )


if __name__ == "__main__":
    main()
