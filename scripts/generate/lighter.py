from object_tools.batch import run_generation_batch
from object_tools.lighter import build_lighter_configs, generate_lighter
from object_tools.measurements import (
    angle,
    length_range,
    length_range_vector,
    length,
    length_vector,
    load_measurement_config,
    run_deterministic_generation,
    validate_keys,
)


INPUT_FILE = "templates/base_lighter.urdf"
OUTPUT_ROOT = "assets/objects/lighter"
DEFAULT_BATCH_CONFIG = "configs/batch/lighter.toml"


def main(num=None, output_root=OUTPUT_ROOT, config_path=None):
    config = load_measurement_config(
        config_path or DEFAULT_BATCH_CONFIG,
        "lighter",
    )
    if num is None:
        validate_keys(
            config,
            required=(
                "body_size",
                "cap_height",
                "joint_lower",
                "joint_upper",
            ),
        )
        exact_config = (
            length_vector(config, "body_size"),
            length(config, "cap_height"),
        )
        joint_limits = (
            angle(config, "joint_lower"),
            angle(config, "joint_upper"),
        )
        def generate_exact(input_path, output_path, object_config):
            return generate_lighter(
                input_path,
                output_path,
                object_config,
                joint_limits,
            )

        run_deterministic_generation(
            INPUT_FILE,
            output_root,
            exact_config,
            generate_exact,
        )
        return

    validate_keys(
        config,
        required=(
            "body_size_ranges",
            "cap_height_range",
            "joint_lower",
            "joint_upper",
        ),
    )
    body_size_ranges = length_range_vector(config, "body_size_ranges")
    cap_height_range = length_range(config, "cap_height_range")
    joint_limits = (
        angle(config, "joint_lower"),
        angle(config, "joint_upper"),
    )
    configs = build_lighter_configs(
        body_size_ranges,
        cap_height_range,
        num,
    )

    def generate_batch(input_path, output_path, object_config):
        return generate_lighter(
            input_path,
            output_path,
            object_config,
            joint_limits,
        )

    run_generation_batch(INPUT_FILE, output_root, configs, generate_batch)


if __name__ == "__main__":
    main(num=1)
