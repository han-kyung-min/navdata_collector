import os
import argparse
from PIL import Image

def check_synced_pngs(base_dir):
    broken_files = []

    for root, dirs, _ in os.walk(base_dir):
        if os.path.basename(root) == "synced":
            for file in os.listdir(root):
                if file.lower().endswith(".png"):
                    path = os.path.join(root, file)
                    try:
                        with Image.open(path) as img:
                            img.verify()
                        print(f"\033[92m[OK]\033[0m {path}")
                    except Exception as e:
                        print(f"\033[91m[BROKEN]\033[0m {path} — {e}")
                        broken_files.append(path)

    print("\nScan complete.")
    if broken_files:
        print(f"\033[91mFound {len(broken_files)} broken PNG file(s):\033[0m")
        for f in broken_files:
            print(f"  - {f}")
    else:
        print("\033[92mAll PNG files in 'synced/' folders are valid.\033[0m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Check PNG files under all 'synced/' folders in base directory.")
    parser.add_argument("base_dir", type=str, help="Base directory containing multiple bag directories.")
    args = parser.parse_args()

    if not os.path.isdir(args.base_dir):
        print(f"\033[91m[ERROR]\033[0m The given path is not a valid directory: {args.base_dir}")
        exit(1)

    check_synced_pngs(args.base_dir)
