import argparse
import importlib
import os

from scripts import OBJECT_TYPES, directory_path


def positive_int(value):
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("--num must be greater than zero")
    return parsed


def config_file(value):
    if not os.path.isfile(value):
        raise argparse.ArgumentTypeError(f"config file does not exist: {value}")
    if os.path.splitext(value)[1].lower() not in {".toml", ".json"}:
        raise argparse.ArgumentTypeError("config must use .toml or .json")
    return value


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate articulated object URDF datasets."
    )
    parser.add_argument(
        "--object",
        required=True,
        choices=OBJECT_TYPES,
        help="Object type to generate.",
    )
    parser.add_argument(
        "--num",
        type=positive_int,
        help="Number of object variants to generate.",
    )
    parser.add_argument(
        "--config",
        type=config_file,
        help=(
            "Batch parameter config when used with --num; exact measurement "
            "config when used without --num."
        ),
    )
    parser.add_argument(
        "--dir",
        required=True,
        type=directory_path,
        help="Output dataset directory path.",
    )
    args = parser.parse_args()
    if args.num is None and args.config is None:
        parser.error("one of --num or --config is required")
    return args


def main():
    args = parse_args()
    generator = importlib.import_module(f"scripts.generate.{args.object}")
    config_path = args.config
    if args.num is not None and config_path is None:
        config_path = f"configs/batch/{args.object}.toml"
    generator.main(
        num=args.num,
        output_root=args.dir,
        config_path=config_path,
    )


if __name__ == "__main__":
    main()
