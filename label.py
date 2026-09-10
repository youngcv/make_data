import argparse
import importlib

from scripts import OBJECT_TYPES, directory_path


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create labeled OBJ meshes from generated URDF objects."
    )
    parser.add_argument(
        "--object",
        required=True,
        choices=OBJECT_TYPES,
        help="Object type to label.",
    )
    parser.add_argument(
        "--dir",
        required=True,
        type=directory_path,
        help="Dataset directory path to label.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    labeler = importlib.import_module(f"scripts.label.{args.object}")
    labeler.main(data_root=args.dir)


if __name__ == "__main__":
    main()
