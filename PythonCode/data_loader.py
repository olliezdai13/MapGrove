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

def load_columns(csv_path: Path) -> Dict[str, List[float]]:
    """Load the requested columns from the provided CSV file."""
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

    return columns


def main() -> None:
    data_files = [
        Path("Data") / "datalog_github1.csv",
        Path("Data") / "datalog_github2.csv",
    ]

    for csv_path in data_files:
        columns = load_columns(csv_path)
        print(f"File: {csv_path}")
        for column_name in COLUMN_NAMES:
            values = columns[column_name]
            sample = ", ".join(str(v) for v in values[:5])
            suffix = "..." if len(values) > 5 else ""
            print(f"  {column_name} ({len(values)} values): [{sample}{suffix}]")
        print()


if __name__ == "__main__":
    main()
