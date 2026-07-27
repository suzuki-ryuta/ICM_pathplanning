#!/usr/bin/env python3
from __future__ import annotations

import configparser
import csv
from datetime import datetime, timezone
import random
import subprocess
import time
import zipfile
from pathlib import Path


# =========================
# settings
# =========================
SAMPLE_COUNT = 30

# Use "fixed" for reproducible samples, or "random" for a different set each run.
SEED_MODE = "fixed"  # "fixed" or "random"
FIXED_SEED = 10455

# CSV output style:
# "columns" -> each line is: x,y,theta
# "tuple"   -> each line is a single CSV cell: (x,y,theta)
CSV_OUTPUT_STYLE = "columns"

# Save to the ICM_Log directory next to this project directory.
OUTPUT_DIR = Path("../ICM_Log")

# None means "<object-name>_test.csv", e.g. T-shape_test.csv.
OUTPUT_FILENAME_OVERRIDE = None

EXCEL_OUTPUT_FILENAME = "実験.xlsx"

# C++ CFreeICS exporter settings.
CLUSTER_EXPORT_BINARY = Path("./Manipulation")
CLUSTER_OUTPUT_DIR = Path("../ICM_Log/initial_clusters")
AUTO_BUILD_CLUSTER_EXPORTER = True
REEXTRACT_CLUSTERS_EACH_RUN = True


ROOT = Path(__file__).resolve().parent
PROBLEM_CONFIG = ROOT / "config" / "ProblemDefine.ini"

SHAPE_SECTIONS = {
    1: "Rectangle",
    2: "LShape",
    3: "Triangle",
    4: "TShape",
}

OUTPUT_SHAPE_NAMES = {
    "Rectangle": "Rectangle",
    "LShape": "L-shape",
    "Triangle": "Triangle",
    "TShape": "T-shape",
}


def read_ini(path: Path) -> configparser.ConfigParser:
    parser = configparser.ConfigParser()
    parser.optionxform = str
    read_files = parser.read(path, encoding="utf-8")
    if not read_files:
        raise FileNotFoundError(f"Could not read config file: {path}")
    return parser


def get_float(parser: configparser.ConfigParser, section: str, key: str) -> float:
    if not parser.has_section(section):
        raise KeyError(f"Missing section [{section}]")
    if not parser.has_option(section, key):
        raise KeyError(f"Missing key {section}.{key}")
    return parser.getfloat(section, key)


def get_int(parser: configparser.ConfigParser, section: str, key: str) -> int:
    return int(get_float(parser, section, key))


def selected_shape_section(problem: configparser.ConfigParser) -> str:
    shape_id = get_int(problem, "object", "shape")
    try:
        return SHAPE_SECTIONS[shape_id]
    except KeyError as exc:
        raise ValueError(f"Unsupported object.shape: {shape_id}") from exc


def make_rng() -> tuple[random.Random, int]:
    if SEED_MODE == "fixed":
        seed = int(FIXED_SEED)
    elif SEED_MODE == "random":
        seed = time.time_ns()
    else:
        raise ValueError('SEED_MODE must be "fixed" or "random"')
    return random.Random(seed), seed


def current_shape_section() -> str:
    problem = read_ini(PROBLEM_CONFIG)
    return selected_shape_section(problem)


def build_cluster_exporter() -> None:
    if not AUTO_BUILD_CLUSTER_EXPORTER:
        return
    subprocess.run(["make", "Manipulation"], cwd=ROOT, check=True)


def export_initial_clusters() -> Path:
    output_dir = (ROOT / CLUSTER_OUTPUT_DIR).resolve()
    manifest_path = output_dir / "manifest.csv"

    build_cluster_exporter()
    if not REEXTRACT_CLUSTERS_EACH_RUN and manifest_path.exists():
        return manifest_path

    binary = ROOT / CLUSTER_EXPORT_BINARY
    if not binary.exists():
        raise FileNotFoundError(
            f"Cluster exporter binary does not exist: {binary}. "
            "Run `make Manipulation` first or set AUTO_BUILD_CLUSTER_EXPORTER=True."
        )

    result = subprocess.run(
        [
            str(binary),
            "--export-initial-clusters",
            "--output-dir",
            str(output_dir),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
    )
    if result.returncode != 0:
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(result.stderr)
        raise RuntimeError(f"CFreeICS cluster export failed: {result.returncode}")

    if not manifest_path.exists():
        raise FileNotFoundError(f"Cluster manifest was not created: {manifest_path}")

    return manifest_path


def load_cluster_manifest(manifest_path: Path) -> list[dict[str, str]]:
    with manifest_path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        clusters = list(reader)
    if not clusters:
        raise RuntimeError("No initial clusters were extracted.")
    return clusters


def print_cluster_summaries(clusters: list[dict[str, str]]) -> None:
    print("Extracted initial pose clusters:")
    for row in clusters:
        print(
            f"  {row['id']}: size={row['size']}, "
            f"x=[{row['min_x']},{row['max_x']}], "
            f"y=[{row['min_y']},{row['max_y']}], "
            f"theta=[{row['min_th']},{row['max_th']}]"
        )


def choose_cluster(clusters: list[dict[str, str]]) -> dict[str, str]:
    valid_ids = {int(row["id"]): row for row in clusters}
    while True:
        raw = input("Select the cluster No. -> ").strip()
        try:
            selected_id = int(raw)
        except ValueError:
            print("Please input an integer cluster id.")
            continue
        if selected_id in valid_ids:
            return valid_ids[selected_id]
        print(f"Invalid cluster id: {selected_id}")


def load_cluster_points(path: Path) -> list[tuple[int, int, int]]:
    points: list[tuple[int, int, int]] = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            if len(row) < 3:
                raise ValueError(f"Invalid cluster row in {path}: {row}")
            points.append((int(row[0]), int(row[1]), int(row[2])))
    if not points:
        raise RuntimeError(f"Selected cluster is empty: {path}")
    return points


def cluster_bounding_box(points: list[tuple[int, int, int]]) -> dict[str, float]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    ths = [p[2] for p in points]
    return {
        "x_min": min(xs),
        "x_max": max(xs),
        "y_min": min(ys),
        "y_max": max(ys),
        "theta_min": min(ths),
        "theta_max": max(ths),
        "theta_upper_exclusive": False,
    }


def bin_index(value: int, lower: float, upper: float, bin_count: int) -> int:
    if bin_count <= 0:
        raise ValueError("bin_count must be positive")
    if upper == lower:
        return 0
    return min(int((value - lower) / (upper - lower) * bin_count), bin_count - 1)


def point_bins(
    point: tuple[int, int, int],
    bbox: dict[str, float],
    bin_count: int,
) -> tuple[int, int, int]:
    return (
        bin_index(point[0], bbox["x_min"], bbox["x_max"], bin_count),
        bin_index(point[1], bbox["y_min"], bbox["y_max"], bin_count),
        bin_index(point[2], bbox["theta_min"], bbox["theta_max"], bin_count),
    )


def score_point(
    bins: tuple[int, int, int],
    used_x: set[int],
    used_y: set[int],
    used_theta: set[int],
) -> int:
    score = 0
    if bins[0] not in used_x:
        score += 1
    if bins[1] not in used_y:
        score += 1
    if bins[2] not in used_theta:
        score += 1
    return score


def sample_from_cluster(
    points: list[tuple[int, int, int]],
    bbox: dict[str, float],
    rng: random.Random,
) -> tuple[list[tuple[int, int, int]], dict[str, object]]:
    unique_points = list(dict.fromkeys(points))
    if len(unique_points) < SAMPLE_COUNT:
        raise RuntimeError(
            f"Selected cluster has only {len(unique_points)} unique points; "
            f"SAMPLE_COUNT={SAMPLE_COUNT} cannot be satisfied."
        )

    bin_count = SAMPLE_COUNT
    bins_by_point = {
        point: point_bins(point, bbox, bin_count)
        for point in unique_points
    }
    remaining = unique_points[:]
    rng.shuffle(remaining)

    selected: list[tuple[int, int, int]] = []
    used_x: set[int] = set()
    used_y: set[int] = set()
    used_theta: set[int] = set()
    score_counts = {3: 0, 2: 0, 1: 0, 0: 0}

    while len(selected) < SAMPLE_COUNT:
        best_score = -1
        best_indices: list[int] = []

        for index, point in enumerate(remaining):
            score = score_point(bins_by_point[point], used_x, used_y, used_theta)
            if score > best_score:
                best_score = score
                best_indices = [index]
            elif score == best_score:
                best_indices.append(index)

        chosen_index = rng.choice(best_indices)
        chosen = remaining.pop(chosen_index)
        selected.append(chosen)
        score_counts[best_score] += 1

        bx, by, btheta = bins_by_point[chosen]
        used_x.add(bx)
        used_y.add(by)
        used_theta.add(btheta)

    diagnostics = {
        "bin_count": bin_count,
        "used_x_bins": len(used_x),
        "used_y_bins": len(used_y),
        "used_theta_bins": len(used_theta),
        "score_counts": score_counts,
    }
    return selected, diagnostics


def generate_poses() -> tuple[
    list[tuple[int, int, int]],
    int,
    dict[str, float],
    dict[str, object],
    str,
    int,
    int,
]:
    manifest_path = export_initial_clusters()
    clusters = load_cluster_manifest(manifest_path)
    print_cluster_summaries(clusters)
    selected_cluster = choose_cluster(clusters)

    cluster_file = Path(selected_cluster["file"])
    if not cluster_file.is_absolute():
        cluster_file = (ROOT / cluster_file).resolve()
    cluster_points = load_cluster_points(cluster_file)
    bbox = cluster_bounding_box(cluster_points)

    rng, seed = make_rng()
    poses, diagnostics = sample_from_cluster(cluster_points, bbox, rng)

    return (
        poses,
        seed,
        bbox,
        diagnostics,
        current_shape_section(),
        int(selected_cluster["id"]),
        len(cluster_points),
    )


def format_pose(pose: tuple[int, int, int]) -> str:
    return f"({pose[0]},{pose[1]},{pose[2]})"


def output_filename(section: str) -> str:
    if OUTPUT_FILENAME_OVERRIDE is not None:
        return OUTPUT_FILENAME_OVERRIDE
    shape_name = OUTPUT_SHAPE_NAMES.get(section, section)
    return f"{shape_name}_test.csv"


def save_csv(poses: list[tuple[int, int, int]], section: str) -> Path:
    output_dir = ROOT / OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / output_filename(section)

    with output_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        if CSV_OUTPUT_STYLE == "columns":
            writer.writerows(poses)
        elif CSV_OUTPUT_STYLE == "tuple":
            writer.writerows([[format_pose(pose)] for pose in poses])
        else:
            raise ValueError('CSV_OUTPUT_STYLE must be "columns" or "tuple"')

    return output_path


def excel_column_name(index: int) -> str:
    name = ""
    while index > 0:
        index, remainder = divmod(index - 1, 26)
        name = chr(ord("A") + remainder) + name
    return name


def make_sheet_xml(poses: list[tuple[int, int, int]]) -> str:
    rows = []
    for row_index, pose in enumerate(poses, start=1):
        cells = []
        for col_index, value in enumerate(pose, start=1):
            cell_ref = f"{excel_column_name(col_index)}{row_index}"
            cells.append(f'<c r="{cell_ref}"><v>{value}</v></c>')
        rows.append(f'<row r="{row_index}">{"".join(cells)}</row>')

    dimension = f"A1:C{len(poses)}" if poses else "A1"
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
        'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
        f'<dimension ref="{dimension}"/>'
        "<sheetViews><sheetView workbookViewId=\"0\"/></sheetViews>"
        '<sheetFormatPr defaultRowHeight="15"/>'
        f"<sheetData>{''.join(rows)}</sheetData>"
        "</worksheet>"
    )


def save_xlsx(poses: list[tuple[int, int, int]]) -> Path:
    output_dir = ROOT / OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / EXCEL_OUTPUT_FILENAME
    created = datetime.now(timezone.utc).replace(microsecond=0).isoformat()

    files = {
        "[Content_Types].xml": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
            '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
            '<Default Extension="xml" ContentType="application/xml"/>'
            '<Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>'
            '<Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>'
            '<Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>'
            '<Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>'
            "</Types>"
        ),
        "_rels/.rels": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>'
            '<Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>'
            '<Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>'
            "</Relationships>"
        ),
        "xl/workbook.xml": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main" '
            'xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">'
            "<sheets>"
            '<sheet name="initial_poses" sheetId="1" r:id="rId1"/>'
            "</sheets>"
            "</workbook>"
        ),
        "xl/_rels/workbook.xml.rels": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>'
            "</Relationships>"
        ),
        "xl/worksheets/sheet1.xml": make_sheet_xml(poses),
        "docProps/core.xml": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties" '
            'xmlns:dc="http://purl.org/dc/elements/1.1/" '
            'xmlns:dcterms="http://purl.org/dc/terms/" '
            'xmlns:dcmitype="http://purl.org/dc/dcmitype/" '
            'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">'
            "<dc:creator>generate_initial_poses.py</dc:creator>"
            "<cp:lastModifiedBy>generate_initial_poses.py</cp:lastModifiedBy>"
            f'<dcterms:created xsi:type="dcterms:W3CDTF">{created}</dcterms:created>'
            f'<dcterms:modified xsi:type="dcterms:W3CDTF">{created}</dcterms:modified>'
            "</cp:coreProperties>"
        ),
        "docProps/app.xml": (
            '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n'
            '<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties" '
            'xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">'
            "<Application>generate_initial_poses.py</Application>"
            "</Properties>"
        ),
    }

    with zipfile.ZipFile(output_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for arcname, content in files.items():
            zf.writestr(arcname, content)

    return output_path


def main() -> None:
    (
        poses,
        seed,
        bbox,
        diagnostics,
        section,
        selected_cluster_id,
        cluster_size,
    ) = generate_poses()
    csv_path = save_csv(poses, section)
    xlsx_path = save_xlsx(poses)

    print(",".join(format_pose(pose) for pose in poses))
    print(f"shape={section}")
    print(f"selected_cluster={selected_cluster_id} (size={cluster_size})")
    print(f"seed={seed} ({SEED_MODE})")
    print(
        "cluster_bbox="
        f"x[{bbox['x_min']},{bbox['x_max']}], "
        f"y[{bbox['y_min']},{bbox['y_max']}], "
        f"theta[{bbox['theta_min']},{bbox['theta_max']}]"
    )
    print(
        "strata_coverage="
        f"x:{diagnostics['used_x_bins']}/{diagnostics['bin_count']}, "
        f"y:{diagnostics['used_y_bins']}/{diagnostics['bin_count']}, "
        f"theta:{diagnostics['used_theta_bins']}/{diagnostics['bin_count']}"
    )
    print(f"score_counts={diagnostics['score_counts']}")
    print(f"csv_saved={csv_path}")
    print(f"xlsx_saved={xlsx_path}")


if __name__ == "__main__":
    main()
