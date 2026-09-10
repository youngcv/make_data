"""Runnable project commands."""

import argparse
import os


OBJECT_TYPES = ("knife", "lighter", "stapler", "tong")


def validate_directory_path(path):
    if not path:
        raise ValueError("--dir cannot be empty")
    return os.path.normpath(os.path.expanduser(path))


def directory_path(value):
    try:
        return validate_directory_path(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc
