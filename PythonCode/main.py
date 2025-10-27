from __future__ import annotations

import csv
from pathlib import Path
from typing import Dict, List


COLUMN_NAMES = [
    "Timestamp",
    "AccX_G",
    "AccY_G",
    "AccZ_G",
    "GyroX_DPS",
    "GyroY_DPS",
    "GyroZ_DPS",
    "Lat",
    "Lon",
    "Altitude",
    "Sats",
    "PlantStat",
]

DATA_FILES = [
    Path("Data") / "datalog_github1.csv",
    Path("Data") / "datalog_github2.csv",
]


def load_columns(csv_paths: List[Path]) -> Dict[Path, Dict[str, List[float]]]:
    """Load the requested columns from each CSV file separately."""
    columns_by_file: Dict[Path, Dict[str, List[float]]] = {}

    for csv_path in csv_paths:
        columns: Dict[str, List[float]] = {column: [] for column in COLUMN_NAMES}

        with csv_path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)

            if reader.fieldnames is None:
                raise ValueError(f"No header row found in {csv_path}")

            missing_columns = [c for c in COLUMN_NAMES if c not in reader.fieldnames]
            if missing_columns:
                missing = ", ".join(missing_columns)
                raise ValueError(f"Missing columns in {csv_path}: {missing}")

            for row in reader:
                for column in COLUMN_NAMES:
                    value = row[column]

                    if column == "Timestamp":
                        columns[column].append(value)
                    elif column in {"Sats", "PlantStat"}:
                        columns[column].append(int(value))
                    else:
                        columns[column].append(float(value))

        columns_by_file[csv_path] = columns

    return columns_by_file


def main() -> None:
    columns_by_file = load_columns(DATA_FILES)

    for csv_path, columns in columns_by_file.items():
        print(f"File: {csv_path}")
        for column_name in COLUMN_NAMES:
            values = columns[column_name]
            sample = ", ".join(str(v) for v in values[:5])
            suffix = "..." if len(values) > 5 else ""
            print(f"  {column_name} ({len(values)} values): [{sample}{suffix}]")
        print()


if __name__ == "__main__":
    main()
