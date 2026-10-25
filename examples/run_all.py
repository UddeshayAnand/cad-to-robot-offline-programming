import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("output/reproduced"))
    args = parser.parse_args()
    if args.output.exists() and (not args.output.is_dir() or any(args.output.iterdir())):
        parser.error("Choose a new or empty output directory")
    fixtures = Path(__file__).resolve().parent / "meshes"
    for name in ("plate", "two_plates", "cylinder"):
        subprocess.run([sys.executable, "-m", "cad_to_robot.cli",
                        str(fixtures / f"{name}.stl"), "--units", "mm", "--z", "0",
                        "--output", str(args.output / name)], check=True)


if __name__ == "__main__":
    main()
