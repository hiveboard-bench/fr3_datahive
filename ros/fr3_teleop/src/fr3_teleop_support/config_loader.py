#!/usr/bin/env python3
"""Read one section of the fr3_teleop YAML config."""

import argparse
import os

import rospkg
import yaml


def load_section(argv, section):
    """Return (config path, `section` mapping) after honouring a --config override."""
    default_path = os.path.join(rospkg.RosPack().get_path("fr3_teleop"), "config", "teleop.yaml")
    pre_parser = argparse.ArgumentParser(add_help=False)
    pre_parser.add_argument("--config", default=default_path)
    pre_args, _ = pre_parser.parse_known_args(argv)

    with open(pre_args.config) as stream:
        document = yaml.safe_load(stream) or {}
    values = document.get(section)
    if not isinstance(values, dict):
        raise SystemExit(f"teleop config {pre_args.config!r} needs a {section!r} mapping")
    return pre_args.config, values
