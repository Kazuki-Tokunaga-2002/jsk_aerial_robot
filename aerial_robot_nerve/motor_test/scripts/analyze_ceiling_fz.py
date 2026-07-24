#!/usr/bin/env python3
import argparse
from pathlib import Path

import numpy as np
import pandas as pd


COLUMN_NAMES = [
    "PWM",
    "fx",
    "fy",
    "fz",
    "f_norm",
    "mx",
    "my",
    "mz",
    "current",
    "State",
]


L_R_DIRS = [
    "l_R_0R3",
    "l_R_0R4",
    "l_R_0R5",
    "l_R_0R7",
    "l_R_1R0",
    "l_R_1R5",
    "l_R_inf",
]

D_R_DIRS = [
    "d_R_0R3",
    "d_R_0R4",
    "d_R_0R5",
    "d_R_0R7",
    "d_R_1R0",
    "d_R_1R5",
    "d_R_2R0",
    "d_R_inf",
]


def extract_condition_label(folder_name: str, prefix: str) -> str:
    """
    Example:
        folder_name = "l_R_0R5", prefix = "l_R_" -> "0R5"
        folder_name = "d_R_inf", prefix = "d_R_" -> "inf"
    """
    if not folder_name.startswith(prefix):
        raise ValueError(f"Invalid folder name: {folder_name}, expected prefix: {prefix}")
    return folder_name.replace(prefix, "")


def read_fz_mean_from_file(file_path: Path, reverse_fz: bool = False):
    """
    1つの motor_test_*.txt について，State == valid の fz 平均を返す。
    valid 行が存在しない場合は np.nan を返す。
    """

    try:
        data = pd.read_csv(
            file_path,
            sep=r"\s+",
            names=COLUMN_NAMES,
            engine="python",
        )
    except Exception as e:
        print(f"[WARNING] Failed to read: {file_path}")
        print(f"          Error: {e}")
        return np.nan, 0

    valid_data = data[data["State"] == "valid"].copy()

    if len(valid_data) == 0:
        return np.nan, 0

    valid_data["fz"] = pd.to_numeric(valid_data["fz"], errors="coerce")

    if reverse_fz:
        valid_data["fz"] *= -1.0

    mean_fz = valid_data["fz"].mean()
    valid_count = valid_data["fz"].count()

    return mean_fz, valid_count


def process_condition_folder(
    pwm: str,
    pwm_folder: Path,
    l_r_dir: str,
    d_r_dir: str,
    reverse_fz: bool = False,
):
    """
    1条件，つまり
        /home/tokunaga/data/1600/l_R_xxx/d_R_yyy/
    の中の motor_test_*.txt をすべて読み，
    ファイルごとの fz 平均 CSV をその条件フォルダ内に保存する。

    返り値:
        condition_mean_fz:
            その条件に存在する各ファイルの mean_fz の平均
    """

    condition_folder = pwm_folder / l_r_dir / d_r_dir

    l_r_label = extract_condition_label(l_r_dir, "l_R_")
    d_r_label = extract_condition_label(d_r_dir, "d_R_")

    if not condition_folder.exists():
        print(f"[WARNING] Folder does not exist: {condition_folder}")
        return np.nan

    motor_files = sorted(condition_folder.glob("motor_test_*.txt"))

    if len(motor_files) == 0:
        print(f"[WARNING] No motor_test_*.txt files in: {condition_folder}")
        return np.nan

    rows = []

    for file_path in motor_files:
        mean_fz, valid_count = read_fz_mean_from_file(file_path, reverse_fz=reverse_fz)

        rows.append(
            {
                "file_name": file_path.name,
                "mean_fz": mean_fz,
                "valid_count": valid_count,
            }
        )

    per_file_df = pd.DataFrame(rows)

    output_name = f"PWM_{pwm}_l_R_{l_r_label}_d_R_{d_r_label}_fz_mean_per_file.csv"
    output_path = condition_folder / output_name

    per_file_df.to_csv(output_path, index=False)

    print(f"[SAVE] {output_path}")

    condition_mean_fz = per_file_df["mean_fz"].mean(skipna=True)

    return condition_mean_fz


def analyze_pwm_folder(pwm_folder_path: str, reverse_fz: bool = False):
    """
    PWMフォルダを指定して，全条件を処理する。

    Example:
        pwm_folder_path = "/home/tokunaga/data/1600"
    """

    pwm_folder = Path(pwm_folder_path).expanduser().resolve()

    if not pwm_folder.exists():
        raise FileNotFoundError(f"PWM folder does not exist: {pwm_folder}")

    pwm = pwm_folder.name

    print(f"PWM folder: {pwm_folder}")
    print(f"PWM: {pwm}")

    mean_table = pd.DataFrame(
        index=[extract_condition_label(l, "l_R_") for l in L_R_DIRS],
        columns=[extract_condition_label(d, "d_R_") for d in D_R_DIRS],
        dtype=float,
    )

    for l_r_dir in L_R_DIRS:
        for d_r_dir in D_R_DIRS:
            l_r_label = extract_condition_label(l_r_dir, "l_R_")
            d_r_label = extract_condition_label(d_r_dir, "d_R_")

            print(f"\n[PROCESS] PWM={pwm}, l_R={l_r_label}, d_R={d_r_label}")

            condition_mean_fz = process_condition_folder(
                pwm=pwm,
                pwm_folder=pwm_folder,
                l_r_dir=l_r_dir,
                d_r_dir=d_r_dir,
                reverse_fz=reverse_fz,
            )

            mean_table.loc[l_r_label, d_r_label] = condition_mean_fz

    mean_table.index.name = "l_R \\ d_R"

    mean_table_path = pwm_folder / f"PWM_{pwm}_fz_mean_table.csv"
    mean_table.to_csv(mean_table_path)

    print(f"\n[SAVE] {mean_table_path}")

    # ratio table
    baseline_l_r = "inf"
    baseline_d_r = "inf"

    baseline_fz = mean_table.loc[baseline_l_r, baseline_d_r]

    if pd.isna(baseline_fz):
        print("\n[WARNING] Baseline fz is NaN.")
        print("          Cannot create ratio table because l_R_inf, d_R_inf value is missing.")
        return

    if baseline_fz == 0:
        print("\n[WARNING] Baseline fz is 0.")
        print("          Cannot create ratio table because division by zero occurs.")
        return

    ratio_table = mean_table / baseline_fz
    ratio_table.index.name = "l_R \\ d_R"

    ratio_table_path = pwm_folder / f"PWM_{pwm}_fz_ratio_table.csv"
    ratio_table.to_csv(ratio_table_path)

    print(f"[SAVE] {ratio_table_path}")
    print(f"\nBaseline: l_R_inf, d_R_inf = {baseline_fz:.6f}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Calculate mean fz for each motor_test file and create fz mean/ratio tables."
    )

    parser.add_argument(
        "pwm_folder",
        help="Path to PWM folder, e.g., /home/tokunaga/data/1600",
    )

    parser.add_argument(
        "--reverse_fz",
        "-r",
        action="store_true",
        help="Reverse sign of fz if the force sensor direction is opposite.",
    )

    args = parser.parse_args()

    analyze_pwm_folder(
        pwm_folder_path=args.pwm_folder,
        reverse_fz=args.reverse_fz,
    )