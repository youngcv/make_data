from object_tools.batch import run_generation_batch
from object_tools.knife import build_knife_configs, generate_knife
from object_tools.measurements import (
    length,
    length_range,
    length_range_vector,
    length_vector,
    load_measurement_config,
    nonnegative_length,
    run_deterministic_generation,
    validate_keys,
)


INPUT_FILE = "templates/base_knife.urdf"
OUTPUT_ROOT = "assets/objects/knife"
DEFAULT_BATCH_CONFIG = "configs/batch/knife.toml"


def main(num=None, output_root=OUTPUT_ROOT, config_path=None):
    config = load_measurement_config(
        config_path or DEFAULT_BATCH_CONFIG,
        "knife",
    )
    if num is None:
        validate_keys(
            config,
            required=(
                "body_size",
                "slider_size",
                "edge_clearance",
                "joint_travel",
            ),
        )
        exact_config = (
            length_vector(config, "body_size"),
            length_vector(config, "slider_size"),
            nonnegative_length(config, "edge_clearance"),
        )
        joint_travel = length(config, "joint_travel")

        def generate_exact(input_path, output_path, object_config):
            return generate_knife(
                input_path,
                output_path,
                object_config,
                joint_travel,
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
            "slider_size_ranges",
            "edge_clearance_range",
            "joint_travel",
        ),
    )
    body_size_ranges = length_range_vector(config, "body_size_ranges")
    slider_size_ranges = length_range_vector(config, "slider_size_ranges")
    edge_clearance_range = length_range(
        config,
        "edge_clearance_range",
        allow_zero=True,
    )
    joint_travel = length(config, "joint_travel")
    configs = build_knife_configs(
        body_size_ranges,
        slider_size_ranges,
        edge_clearance_range,
        num,
        joint_travel,
    )

    def generate_batch(input_path, output_path, object_config):
        return generate_knife(
            input_path,
            output_path,
            object_config,
            joint_travel,
        )

    run_generation_batch(INPUT_FILE, output_root, configs, generate_batch)


if __name__ == "__main__":
    main(num=1)
