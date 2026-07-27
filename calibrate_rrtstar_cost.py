#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


DOF = 6
WEIGHTS = (1.0, 1.0, 1.0, 1.0, 1.0, 1.0)
DEFAULT_TOLERANCE_SEC = 0.2
DEFAULT_PATH_DIR = Path(__file__).resolve().parent / "../ICM_Log/path"


@dataclass(frozen=True)
class PathSample:
    name: str
    csv_path: Path
    real_time: float
    node_count: int
    edge_count: int
    motion_sum: float


@dataclass(frozen=True)
class FitResult:
    omega: float
    time_loss: float
    alpha: float
    sse: float
    constrained: bool


def normalize_csv_name(raw_name: str) -> str:
    name = raw_name.strip()
    if name.lower().endswith(".csv"):
        name = name[:-4]
    return name


def read_real_time(prompt: str) -> float:
    while True:
        raw = input(prompt).strip()
        try:
            value = float(raw)
        except ValueError:
            print("数値で入力してください。例: 12.34")
            continue
        if value < 0.0:
            print("時間は0以上で入力してください。")
            continue
        return value


def read_nodes(csv_path: Path) -> list[tuple[float, ...]]:
    nodes: list[tuple[float, ...]] = []

    with csv_path.open(newline="", encoding="utf-8-sig") as file:
        reader = csv.reader(file)
        for line_no, row in enumerate(reader, start=1):
            if not row or all(cell.strip() == "" for cell in row):
                continue
            if len(row) < DOF:
                raise ValueError(
                    f"{csv_path} の {line_no} 行目は {len(row)} 列です。"
                    f"{DOF} 列必要です。"
                )
            try:
                nodes.append(tuple(float(row[i].strip()) for i in range(DOF)))
            except ValueError as exc:
                raise ValueError(
                    f"{csv_path} の {line_no} 行目に数値でない値があります: {row[:DOF]}"
                ) from exc

    if len(nodes) < 2:
        raise ValueError(f"{csv_path} は2ノード以上必要です。読み取れたノード数: {len(nodes)}")

    return nodes


def edge_motion(q1: Iterable[float], q2: Iterable[float]) -> float:
    return max(weight * abs(a - b) for weight, a, b in zip(WEIGHTS, q1, q2))


def motion_sum(nodes: list[tuple[float, ...]]) -> float:
    return sum(edge_motion(nodes[i - 1], nodes[i]) for i in range(1, len(nodes)))


def make_sample(path_dir: Path, file_name: str, real_time: float) -> PathSample:
    csv_path = path_dir / f"{file_name}.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"CSVが見つかりません: {csv_path}")

    nodes = read_nodes(csv_path)
    return PathSample(
        name=file_name,
        csv_path=csv_path,
        real_time=real_time,
        node_count=len(nodes),
        edge_count=len(nodes) - 1,
        motion_sum=motion_sum(nodes),
    )


def prompt_samples(path_dir: Path) -> list[PathSample]:
    samples: list[PathSample] = []
    print(f"CSVフォルダ: {path_dir}")

    while True:
        allow_finish = len(samples) >= 1
        suffix = " / finish と入力すると，実行します．" if allow_finish else ""
        raw_name = input(f"{len(samples) + 1}つ目のファイル名(.csvなし{suffix}): ")
        if allow_finish and raw_name.strip().lower() == "finish":
            break

        file_name = normalize_csv_name(raw_name)
        if not file_name:
            print("ファイル名を入力してください。")
            continue

        real_time = read_real_time(f"{file_name}.csv の実機時間[s]: ")
        try:
            sample = make_sample(path_dir, file_name, real_time)
        except (FileNotFoundError, ValueError) as exc:
            print(f"[ERROR] {exc}")
            continue

        samples.append(sample)
        print(
            f"[OK] {file_name}.csv: nodes={sample.node_count}, "
            f"edges={sample.edge_count}, motion_sum={sample.motion_sum:.6f}"
        )

    return samples


def squared_error(samples: list[PathSample], alpha: float, time_loss: float) -> float:
    return sum(
        (sample.motion_sum * alpha + sample.edge_count * time_loss - sample.real_time) ** 2
        for sample in samples
    )


def fit_unconstrained(samples: list[PathSample]) -> tuple[float, float]:
    ss = sum(sample.motion_sum * sample.motion_sum for sample in samples)
    sm = sum(sample.motion_sum * sample.edge_count for sample in samples)
    mm = sum(sample.edge_count * sample.edge_count for sample in samples)
    sy = sum(sample.motion_sum * sample.real_time for sample in samples)
    my = sum(sample.edge_count * sample.real_time for sample in samples)

    det = ss * mm - sm * sm
    scale = max(ss * mm, 1.0)
    if abs(det) <= 1e-12 * scale:
        raise ValueError(
            "motion_sum と edge_count の比がほぼ同じため、"
            "omega と Timeloss を分離して推定できません。"
            "ノード数や動き量が異なるCSVを追加してください。"
        )

    alpha = (sy * mm - my * sm) / det
    time_loss = (ss * my - sm * sy) / det
    return alpha, time_loss


def fit_nonnegative(samples: list[PathSample]) -> FitResult:
    candidates: list[tuple[float, float, bool]] = []

    alpha, time_loss = fit_unconstrained(samples)
    if alpha > 0.0 and time_loss >= 0.0:
        candidates.append((alpha, time_loss, False))

    ss = sum(sample.motion_sum * sample.motion_sum for sample in samples)
    sy = sum(sample.motion_sum * sample.real_time for sample in samples)
    if ss > 0.0:
        candidates.append((max(sy / ss, 0.0), 0.0, True))

    mm = sum(sample.edge_count * sample.edge_count for sample in samples)
    my = sum(sample.edge_count * sample.real_time for sample in samples)
    if mm > 0.0:
        candidates.append((0.0, max(my / mm, 0.0), True))

    if not candidates:
        raise ValueError("有効な推定候補を作れませんでした。CSVと実機時間を確認してください。")

    best_alpha, best_time_loss, constrained = min(
        candidates,
        key=lambda candidate: squared_error(samples, candidate[0], candidate[1]),
    )
    if best_alpha <= 0.0:
        raise ValueError(
            "推定された 1/omega が0以下です。omegaを有限の正値として推定できません。"
        )

    return FitResult(
        omega=1.0 / best_alpha,
        time_loss=best_time_loss,
        alpha=best_alpha,
        sse=squared_error(samples, best_alpha, best_time_loss),
        constrained=constrained,
    )


def predicted_time(sample: PathSample, result: FitResult) -> float:
    return sample.motion_sum / result.omega + sample.edge_count * result.time_loss


def print_report(samples: list[PathSample], result: FitResult, tolerance: float) -> None:
    print("\n=== RRTStar cost calibration log ===")
    print(
        f"[FIT] samples={len(samples)}, omega={result.omega:.10g}, "
        f"time_loss={result.time_loss:.10g}, tolerance={tolerance:.3f}s"
    )
    if result.constrained:
        print("[WARN] 非負制約を使った推定です。元データのばらつきが大きい可能性があります。")

    max_abs_error = 0.0
    for sample in samples:
        sim_time = predicted_time(sample, result)
        error = sim_time - sample.real_time
        max_abs_error = max(max_abs_error, abs(error))
        status = "OK" if abs(error) <= tolerance else "NG"
        print(
            f"[DATA] file={sample.name}.csv, nodes={sample.node_count}, "
            f"edges={sample.edge_count}, motion_sum={sample.motion_sum:.6f}, "
            f"real_time={sample.real_time:.6f}s, sim_time={sim_time:.6f}s, "
            f"error={error:+.6f}s, status={status}"
        )

    rmse = math.sqrt(result.sse / len(samples))
    print(f"[ERROR] max_abs_error={max_abs_error:.6f}s, rmse={rmse:.6f}s")
    if max_abs_error <= tolerance:
        print(f"[RESULT] OK: 全データが {tolerance:.3f}s 以内です。")
    else:
        print(
            f"[RESULT] NG: {tolerance:.3f}s を超えるデータがあります。"
            "CSVと実機時間を増やす、または外れ値を確認してください。"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="RRTStar の omega と Timeloss を実機時間から推定します。"
    )
    parser.add_argument(
        "--path-dir",
        type=Path,
        default=DEFAULT_PATH_DIR,
        help="CSVファイルが入っているフォルダ。既定値: ../ICM_Log/path",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=DEFAULT_TOLERANCE_SEC,
        help="OK判定に使う許容誤差[s]。既定値: 0.2",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    path_dir = args.path_dir.expanduser().resolve()

    try:
        samples = prompt_samples(path_dir)
        if len(samples) < 2:
            print("omega と Timeloss の2変数を推定するため、最低2つのCSVが必要です。")
            return 1
        result = fit_nonnegative(samples)
    except KeyboardInterrupt:
        print("\n中断しました。")
        return 130
    except (EOFError, FileNotFoundError, ValueError) as exc:
        print(f"[ERROR] {exc}")
        return 1

    print_report(samples, result, args.tolerance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
