#!/usr/bin/env python3
from __future__ import annotations

import configparser
import csv
from pathlib import Path
from typing import Iterable


DEFAULT_DOF = 6
DEFAULT_PATH_DIR = Path(__file__).resolve().parent / "../ICM_Log/path"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parent / "config/ProblemDefine.ini"


def normalize_csv_name(raw_name: str) -> str:
    name = raw_name.strip()
    if name.lower().endswith(".csv"):
        name = name[:-4]
    return name


def read_rrtstar_config(config_path: Path) -> tuple[int, float, float]:
    if not config_path.exists():
        raise FileNotFoundError(f"設定ファイルが見つかりません: {config_path}")

    parser = configparser.ConfigParser()
    parser.read(config_path, encoding="utf-8")

    try:
        dof = parser.getint("RRTStar", "d", fallback=DEFAULT_DOF)
        omega = parser.getfloat("RRTStar", "omega")
        time_loss = parser.getfloat("RRTStar", "Timeloss")
    except (configparser.Error, ValueError) as exc:
        raise ValueError(
            "config/ProblemDefine.ini の [RRTStar] に "
            "omega と Timeloss を数値で設定してください。"
        ) from exc

    if dof <= 0:
        raise ValueError("RRTStar.d は1以上にしてください。")
    if omega <= 0.0:
        raise ValueError("RRTStar.omega は0より大きい値にしてください。")

    return dof, omega, time_loss


def read_nodes(csv_path: Path, dof: int) -> list[tuple[float, ...]]:
    nodes: list[tuple[float, ...]] = []

    with csv_path.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.reader(file)
        for line_no, row in enumerate(reader, start=1):
            if not row or all(cell.strip() == "" for cell in row):
                continue
            if len(row) < dof:
                raise ValueError(
                    f"{csv_path} の {line_no} 行目は {len(row)} 列です。"
                    f"{dof} 列必要です。"
                )
            try:
                nodes.append(tuple(float(row[i].strip()) for i in range(dof)))
            except ValueError as exc:
                raise ValueError(
                    f"{csv_path} の {line_no} 行目に数値でない値があります: {row[:dof]}"
                ) from exc

    if len(nodes) < 2:
        raise ValueError(f"{csv_path} は2ノード以上必要です。読み取れたノード数: {len(nodes)}")

    return nodes


def edge_motion(q1: Iterable[float], q2: Iterable[float]) -> float:
    return max(abs(a - b) for a, b in zip(q1, q2))


def calculate_total_cost(
    nodes: list[tuple[float, ...]],
    omega: float,
    time_loss: float,
) -> float:
    total_cost = 0.0
    for i in range(1, len(nodes)):
        total_cost += edge_motion(nodes[i - 1], nodes[i]) / omega + time_loss
    return total_cost


def prompt_file_name() -> str:
    while True:
        file_name = normalize_csv_name(input("ファイル名(.csvなし): "))
        if file_name:
            return file_name
        print("ファイル名を入力してください。")


def main() -> int:
    path_dir = DEFAULT_PATH_DIR.resolve()
    config_path = DEFAULT_CONFIG_PATH.resolve()

    try:
        dof, omega, time_loss = read_rrtstar_config(config_path)
        file_name = prompt_file_name()
        csv_path = path_dir / f"{file_name}.csv"
        if not csv_path.exists():
            raise FileNotFoundError(f"CSVが見つかりません: {csv_path}")

        nodes = read_nodes(csv_path, dof)
        total_cost = calculate_total_cost(nodes, omega, time_loss)
    except KeyboardInterrupt:
        print("\n中断しました。")
        return 130
    except (EOFError, FileNotFoundError, ValueError) as exc:
        print(f"[ERROR] {exc}")
        return 1

    print(f"総コストは{total_cost:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
