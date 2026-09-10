import ast
import json
import math
import os

try:
    import tomllib
except ModuleNotFoundError:
    try:
        import tomli as tomllib
    except ModuleNotFoundError:
        tomllib = None

from object_tools.batch import append_generation


LENGTH_SCALES = {
    "m": 1.0,
    "cm": 0.01,
    "mm": 0.001,
}
ANGLE_SCALES = {
    "rad": 1.0,
    "deg": math.pi / 180.0,
}
METADATA_KEYS = {"object", "units", "angle_units"}


def load_measurement_config(path, expected_object):
    extension = os.path.splitext(path)[1].lower()
    if extension == ".toml":
        if tomllib is not None:
            with open(path, "rb") as handle:
                config = tomllib.load(handle)
        else:
            with open(path, encoding="utf-8") as handle:
                config = _load_flat_toml(handle, path)
    elif extension == ".json":
        with open(path, encoding="utf-8") as handle:
            config = json.load(handle)
    else:
        raise ValueError("config must use .toml or .json")

    if not isinstance(config, dict):
        raise ValueError("config root must be a table/object")

    configured_object = config.get("object")
    if configured_object is not None and configured_object != expected_object:
        raise ValueError(
            f"config object is '{configured_object}', expected '{expected_object}'"
        )

    length_unit = config.get("units", "m")
    angle_unit = config.get("angle_units", "rad")
    if length_unit not in LENGTH_SCALES:
        raise ValueError(f"unsupported length unit: {length_unit}")
    if angle_unit not in ANGLE_SCALES:
        raise ValueError(f"unsupported angle unit: {angle_unit}")

    return config


def validate_keys(config, required, optional=()):
    required = set(required)
    allowed = required | set(optional) | METADATA_KEYS
    missing = sorted(required - config.keys())
    unknown = sorted(config.keys() - allowed)
    if missing:
        raise ValueError(f"missing required config fields: {', '.join(missing)}")
    if unknown:
        raise ValueError(f"unknown config fields: {', '.join(unknown)}")


def length(config, key):
    return positive_number(config, key) * LENGTH_SCALES[config.get("units", "m")]


def nonnegative_length(config, key):
    value = _number(config.get(key), key)
    if value < 0.0:
        raise ValueError(f"{key} must be zero or greater")
    return value * LENGTH_SCALES[config.get("units", "m")]


def length_vector(config, key):
    value = config.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{key} must contain exactly three numbers")
    scale = LENGTH_SCALES[config.get("units", "m")]
    converted = tuple(
        _number(item, f"{key}[{index}]") * scale
        for index, item in enumerate(value)
    )
    if any(item <= 0.0 for item in converted):
        raise ValueError(f"all {key} dimensions must be greater than zero")
    return converted


def length_range(config, key, allow_zero=False):
    low, high = _range(config, key)
    scale = LENGTH_SCALES[config.get("units", "m")]
    low *= scale
    high *= scale
    minimum = 0.0 if allow_zero else 1e-15
    if low < minimum:
        qualifier = "zero or greater" if allow_zero else "greater than zero"
        raise ValueError(f"{key} values must be {qualifier}")
    return low, high


def length_range_vector(config, key):
    value = config.get(key)
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{key} must contain exactly three [min, max] ranges")
    scale = LENGTH_SCALES[config.get("units", "m")]
    ranges = []
    for index, item in enumerate(value):
        low, high = _range_value(item, f"{key}[{index}]")
        low *= scale
        high *= scale
        if low <= 0.0:
            raise ValueError(f"{key}[{index}] values must be greater than zero")
        ranges.append((low, high))
    return ranges


def angle(config, key):
    return _number(config.get(key), key) * ANGLE_SCALES[config.get("angle_units", "rad")]


def angle_range(config, key):
    low, high = _range(config, key)
    scale = ANGLE_SCALES[config.get("angle_units", "rad")]
    return low * scale, high * scale


def run_deterministic_generation(
    input_file,
    output_root,
    generator_config,
    generate_one,
):
    object_id, _ = append_generation(
        input_file,
        output_root,
        generator_config,
        generate_one,
    )
    return object_id


def hinged_pair_parameters(config):
    validate_keys(
        config,
        required=("link_size", "joint_lower", "joint_upper"),
    )
    return (
        length_vector(config, "link_size"),
        angle(config, "joint_lower"),
        angle(config, "joint_upper"),
    )


def positive_number(config, key):
    value = _number(config.get(key), key)
    if value <= 0.0:
        raise ValueError(f"{key} must be greater than zero")
    return value


def _number(value, field_name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number")
    if not math.isfinite(value):
        raise ValueError(f"{field_name} must be finite")
    return float(value)


def _range(config, key):
    return _range_value(config.get(key), key)


def _range_value(value, field_name):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{field_name} must contain [min, max]")
    low = _number(value[0], f"{field_name}[0]")
    high = _number(value[1], f"{field_name}[1]")
    if low > high:
        raise ValueError(f"{field_name} minimum cannot exceed maximum")
    return low, high


def _load_flat_toml(handle, path):
    """Parse the flat scalar/list TOML subset used by measurement configs."""
    config = {}
    pending_key = None
    pending_value = ""
    pending_line = None

    for line_number, raw_line in enumerate(handle, start=1):
        line = _strip_toml_comment(raw_line).strip()
        if not line:
            continue

        if pending_key is not None:
            pending_value = f"{pending_value} {line}"
            if not _toml_value_complete(pending_value):
                continue
            _store_flat_toml_value(
                config,
                pending_key,
                pending_value,
                path,
                pending_line,
            )
            pending_key = None
            pending_value = ""
            pending_line = None
            continue

        if line.startswith("[") and "=" not in line:
            raise ValueError(
                f"{path}:{line_number}: TOML sections require Python 3.11+ "
                "or the optional 'tomli' package"
            )
        if "=" not in line:
            raise ValueError(f"{path}:{line_number}: expected 'key = value'")

        key, raw_value = (part.strip() for part in line.split("=", 1))
        if not key or not key.replace("_", "").isalnum():
            raise ValueError(f"{path}:{line_number}: invalid key '{key}'")
        if key in config:
            raise ValueError(f"{path}:{line_number}: duplicate key '{key}'")

        if not _toml_value_complete(raw_value):
            pending_key = key
            pending_value = raw_value
            pending_line = line_number
            continue
        _store_flat_toml_value(config, key, raw_value, path, line_number)

    if pending_key is not None:
        raise ValueError(
            f"{path}:{pending_line}: unterminated TOML value for '{pending_key}'"
        )
    return config


def _store_flat_toml_value(config, key, raw_value, path, line_number):
    normalized_value = {
        "true": "True",
        "false": "False",
    }.get(raw_value, raw_value)
    try:
        config[key] = ast.literal_eval(normalized_value)
    except (SyntaxError, ValueError) as exc:
        raise ValueError(
            f"{path}:{line_number}: unsupported TOML value for '{key}'"
        ) from exc


def _toml_value_complete(value):
    quote = None
    escaped = False
    bracket_depth = 0
    for character in value:
        if escaped:
            escaped = False
            continue
        if character == "\\" and quote == '"':
            escaped = True
            continue
        if character in {'"', "'"}:
            quote = None if quote == character else character if quote is None else quote
            continue
        if quote is not None:
            continue
        if character in "[{(":
            bracket_depth += 1
        elif character in "]})":
            bracket_depth -= 1
            if bracket_depth < 0:
                return True
    return quote is None and bracket_depth == 0


def _strip_toml_comment(line):
    quote = None
    escaped = False
    for index, character in enumerate(line):
        if escaped:
            escaped = False
            continue
        if character == "\\" and quote == '"':
            escaped = True
            continue
        if character in {'"', "'"}:
            quote = None if quote == character else character if quote is None else quote
            continue
        if character == "#" and quote is None:
            return line[:index]
    return line
