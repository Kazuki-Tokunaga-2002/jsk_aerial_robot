#!/usr/bin/env python3
import argparse
from pathlib import Path


def delete_generated_files(pwm_folder_path: str, dry_run: bool = False):
    pwm_folder = Path(pwm_folder_path).expanduser().resolve()

    if not pwm_folder.exists():
        raise FileNotFoundError(f"PWM folder does not exist: {pwm_folder}")

    pwm = pwm_folder.name

    target_patterns = [
        f"PWM_{pwm}_fz_mean_table.csv",
        f"PWM_{pwm}_fz_ratio_table.csv",
        f"PWM_{pwm}_l_R_*_d_R_*_fz_mean_per_file.csv",
    ]

    files_to_delete = []

    for pattern in target_patterns:
        files_to_delete.extend(pwm_folder.rglob(pattern))

    files_to_delete = sorted(set(files_to_delete))

    if len(files_to_delete) == 0:
        print("No generated CSV files found.")
        return

    print("Files to delete:")
    for file_path in files_to_delete:
        print(f"  {file_path}")

    if dry_run:
        print("\nDry run mode: no files were deleted.")
        return

    for file_path in files_to_delete:
        try:
            file_path.unlink()
            print(f"[DELETE] {file_path}")
        except Exception as e:
            print(f"[WARNING] Failed to delete: {file_path}")
            print(f"          Error: {e}")

    print(f"\nDone. Deleted {len(files_to_delete)} files.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Delete generated fz mean/ratio CSV files."
    )

    parser.add_argument(
        "pwm_folder",
        help="Path to PWM folder, e.g., /home/tokunaga/data/1600",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show files that would be deleted, without deleting them.",
    )

    args = parser.parse_args()

    delete_generated_files(
        pwm_folder_path=args.pwm_folder,
        dry_run=args.dry_run,
    )