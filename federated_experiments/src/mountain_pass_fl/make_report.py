from __future__ import annotations

import argparse
import json
from pathlib import Path

import pandas as pd

from .run_all import write_report


def _read_csv_if_exists(path: Path) -> pd.DataFrame | None:
    return pd.read_csv(path) if path.exists() else None


def main() -> None:
    parser = argparse.ArgumentParser(description="Regenerate REPORT.md from existing outputs")
    parser.add_argument("--out-dir", type=str, default="outputs", help="Results directory")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    segments = _read_csv_if_exists(out_dir / "segments.csv")
    all_routes = _read_csv_if_exists(out_dir / "all_route_predictions.csv")
    all_summary = _read_csv_if_exists(out_dir / "all_summary.csv")
    risk_summary = _read_csv_if_exists(out_dir / "risk" / "risk_summary.csv")

    if segments is None:
        raise FileNotFoundError(f"{out_dir / 'segments.csv'} does not exist")
    if all_routes is None:
        raise FileNotFoundError(f"{out_dir / 'all_route_predictions.csv'} does not exist")
    if all_summary is None:
        all_summary = pd.DataFrame()

    split_path = out_dir / "client_split.json"
    if split_path.exists():
        split_info = json.loads(split_path.read_text(encoding="utf-8"))
    else:
        split_info = {"train": [], "val": [], "test": []}

    skipped_path = out_dir / "skipped_experiments.json"
    if skipped_path.exists():
        skipped = json.loads(skipped_path.read_text(encoding="utf-8"))
    else:
        skipped = []

    write_report(
        out_dir,
        segments=segments,
        all_route_predictions=all_routes,
        all_summary=all_summary,
        risk_summary=risk_summary,
        skipped=skipped,
        split_info=split_info,
    )
    print(f"Report regenerated: {(out_dir / 'REPORT.md').resolve()}")


if __name__ == "__main__":
    main()
