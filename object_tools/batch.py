import json
import os


def run_generation_batch(
    input_file,
    output_root,
    configs,
    generate_one,
    output_file="mobility.urdf",
):
    """Generate numbered objects and write the common size metadata files."""
    link_sizes_by_id = {}
    os.makedirs(output_root, exist_ok=True)

    for index, config in enumerate(configs):
        object_id = f"{index:03d}"
        output_dir = os.path.join(output_root, object_id)
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, output_file)

        link_sizes = generate_one(input_file, output_path, config)
        link_sizes_by_id[object_id] = link_sizes

    _write_json(os.path.join(output_root, "lbx.json"), link_sizes_by_id)


def append_generation(
    input_file,
    output_root,
    config,
    generate_one,
    output_file="mobility.urdf",
):
    """Append one numbered object and merge its metadata with prior entries."""
    os.makedirs(output_root, exist_ok=True)
    link_sizes_path = os.path.join(output_root, "lbx.json")
    link_sizes_by_id = _read_json(link_sizes_path)

    existing_ids = {
        name
        for name in os.listdir(output_root)
        if name.isdigit() and os.path.isdir(os.path.join(output_root, name))
    }
    existing_ids.update(key for key in link_sizes_by_id if key.isdigit())
    next_index = max((int(object_id) for object_id in existing_ids), default=-1) + 1
    object_id = f"{next_index:03d}"

    output_dir = os.path.join(output_root, object_id)
    os.makedirs(output_dir, exist_ok=False)
    output_path = os.path.join(output_dir, output_file)
    link_sizes = generate_one(input_file, output_path, config)

    link_sizes_by_id[object_id] = link_sizes
    _write_json(link_sizes_path, link_sizes_by_id)
    return object_id, output_dir


def _read_json(path):
    if not os.path.exists(path):
        return {}
    with open(path) as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"metadata file must contain a JSON object: {path}")
    return data


def _write_json(path, data):
    with open(path, "w") as handle:
        json.dump(data, handle, indent=2)
