from object_tools.batch import run_generation_batch
from object_tools.hinged_pair import (
    build_equal_link_configs,
    generate_equal_hinged_pair,
)
from object_tools.measurements import (
    angle,
    angle_range,
    hinged_pair_parameters,
    length_range_vector,
    load_measurement_config,
    run_deterministic_generation,
    validate_keys,
)


INPUT_FILE = "templates/base_stapler.urdf"
OUTPUT_ROOT = "assets/objects/stapler"
DEFAULT_BATCH_CONFIG = "configs/batch/stapler.toml"
MIN_Z_FILTER_JOINT_OFFSET = 0.3


def main(num=None, output_root=OUTPUT_ROOT, config_path=None):
    config = load_measurement_config(
        config_path or DEFAULT_BATCH_CONFIG,
        "stapler",
    )
    if num is None:
        link_size, joint_lower, joint_upper = hinged_pair_parameters(config)

        def generate_exact(input_path, output_path, object_config):
            return generate_equal_hinged_pair(
                input_path,
                output_path,
                object_config,
                joint_upper,
            )

        run_deterministic_generation(
            INPUT_FILE,
            output_root,
            (link_size, joint_lower),
            generate_exact,
        )
        return

    validate_keys(
        config,
        required=("link_size_ranges", "joint_lower_range", "joint_upper"),
    )
    size_ranges = length_range_vector(config, "link_size_ranges")
    joint_lower_range = angle_range(config, "joint_lower_range")
    joint_upper = angle(config, "joint_upper")
    configs = build_equal_link_configs(
        size_ranges,
        joint_lower_range,
        num,
        MIN_Z_FILTER_JOINT_OFFSET,
    )

    def generate_batch(input_path, output_path, object_config):
        return generate_equal_hinged_pair(
            input_path,
            output_path,
            object_config,
            joint_upper,
        )

    run_generation_batch(INPUT_FILE, output_root, configs, generate_batch)


if __name__ == "__main__":
    main(num=1)
