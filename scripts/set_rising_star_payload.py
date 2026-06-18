#!/usr/bin/env python3
"""Set Rising Star payload inertial values in the Gazebo Classic SDF.

Gazebo Classic reads link mass/inertia when the model is spawned. This helper
updates the payload_link SDF inertial block before SITL starts; it does not try
to change physics at runtime.
"""

import argparse
import re
from pathlib import Path


LB_TO_KG = 0.45359237
IN_TO_M = 0.0254


def default_sdf_path() -> Path:
    return Path(__file__).resolve().parents[1] / "models" / "rising_star" / "rising_star.sdf"


def read_float_tag(text: str, tag: str) -> float:
    match = re.search(rf"<{tag}>\s*([-+0-9.eE]+)\s*</{tag}>", text)

    if not match:
        raise RuntimeError(f"Could not find <{tag}> in SDF")

    return float(match.group(1))


def replace_first_tag(text: str, tag: str, value: str) -> str:
    pattern = rf"(<{tag}>\s*)[-+0-9.eE]+(\s*</{tag}>)"
    new_text, count = re.subn(pattern, rf"\g<1>{value}\2", text, count=1)

    if count != 1:
        raise RuntimeError(f"Could not replace first <{tag}> value")

    return new_text


def replace_payload_inertial(text: str, mass_kg: float, ixx_iyy: float, izz: float) -> str:
    block_pattern = r'(<link name="payload_link">.*?</link>)'
    block_match = re.search(block_pattern, text, flags=re.DOTALL)

    if not block_match:
        raise RuntimeError('Could not find <link name="payload_link"> block')

    block = block_match.group(1)
    block = replace_first_tag(block, "mass", f"{mass_kg:.10f}")
    block = replace_first_tag(block, "ixx", f"{ixx_iyy:.8f}")
    block = replace_first_tag(block, "iyy", f"{ixx_iyy:.8f}")
    block = replace_first_tag(block, "izz", f"{izz:.8f}")

    return text[:block_match.start(1)] + block + text[block_match.end(1):]


def payload_inertia(payload_lb: float, outer_diameter_in: float, inner_diameter_in: float,
                    thickness_in: float, dummy_mass_kg: float):
    mass_kg = payload_lb * LB_TO_KG if payload_lb > 0.0 else dummy_mass_kg

    outer_radius_m = max(0.001, 0.5 * outer_diameter_in * IN_TO_M)
    inner_radius_m = max(0.0, 0.5 * inner_diameter_in * IN_TO_M)
    thickness_m = max(0.001, thickness_in * IN_TO_M)

    radius_term = outer_radius_m * outer_radius_m + inner_radius_m * inner_radius_m
    ixx_iyy = (mass_kg / 12.0) * (3.0 * radius_term + thickness_m * thickness_m)
    izz = 0.5 * mass_kg * radius_term

    return mass_kg, ixx_iyy, izz


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Update Rising Star payload_link SDF mass/inertia before SITL starts.")
    parser.add_argument("payload_lb", type=float, help="Payload mass in pounds-mass, for example 0, 200, 360")
    parser.add_argument("--sdf", type=Path, default=default_sdf_path(), help="Path to rising_star.sdf")
    parser.add_argument("--dummy-mass-kg", type=float, default=0.5,
                        help="Small positive payload_link mass used when payload_lb is 0")
    parser.add_argument("--dry-run", action="store_true", help="Print calculated values without writing the SDF")
    args = parser.parse_args()

    if args.payload_lb < 0.0:
        raise SystemExit("payload_lb must be >= 0")

    if args.dummy_mass_kg <= 0.0:
        raise SystemExit("--dummy-mass-kg must be > 0")

    sdf_path = args.sdf.resolve()
    text = sdf_path.read_text()

    outer_diameter_in = read_float_tag(text, "payloadOuterDiameterIn")
    inner_diameter_in = read_float_tag(text, "payloadInnerDiameterIn")
    thickness_in = read_float_tag(text, "payloadThicknessIn")

    mass_kg, ixx_iyy, izz = payload_inertia(
        args.payload_lb,
        outer_diameter_in,
        inner_diameter_in,
        thickness_in,
        args.dummy_mass_kg)

    print(f"payload_lb: {args.payload_lb:.3f}")
    print(f"payload_link mass_kg: {mass_kg:.10f}")
    print(f"payload_link ixx/iyy: {ixx_iyy:.8f}")
    print(f"payload_link izz: {izz:.8f}")
    print(f"SDF: {sdf_path}")

    if args.dry_run:
        return 0

    text = replace_payload_inertial(text, mass_kg, ixx_iyy, izz)
    text = replace_first_tag(text, "payloadMassLb", f"{args.payload_lb:.1f}")
    sdf_path.write_text(text)
    print(f"Updated {sdf_path}. Restart Gazebo/PX4 SITL before testing.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
