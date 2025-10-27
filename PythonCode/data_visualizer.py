from __future__ import annotations

from pathlib import Path
from typing import Dict, Iterable, List, Sequence

import matplotlib.pyplot as plt


def plot_columns(
    columns: Dict[str, List[float]],
    timestamp_column: str = "Timestamp",
    selected_columns: Sequence[str] | None = None,
    title: str | None = None,
    output_path: str | Path = "plot.png",
) -> None:
    """
    Plot the selected columns against the timestamp column.

    Args:
        columns: Mapping of column name to list of values.
        timestamp_column: Name of the column to use as the X axis.
        selected_columns: Columns to plot on the Y axis. If None, all numeric columns
            except the timestamp column are plotted.
        title: Optional plot title.
    """
    if timestamp_column not in columns:
        raise ValueError(f"Timestamp column '{timestamp_column}' not present in data.")

    timestamps = _to_float_sequence(columns[timestamp_column])

    if selected_columns is None:
        selected_columns = [
            column
            for column in columns
            if column != timestamp_column and _is_numeric_sequence(columns[column])
        ]

    if not selected_columns:
        raise ValueError("No columns selected for plotting.")

    fig, ax = plt.subplots()

    for column in selected_columns:
        if column not in columns:
            raise ValueError(f"Column '{column}' not present in data.")

        values = _to_float_sequence(columns[column])
        ax.plot(timestamps, values, label=column)

    ax.set_xlabel(timestamp_column)
    ax.set_ylabel("Value")
    ax.set_title(title or "Data Visualization")
    ax.legend()
    ax.grid(True)
    ax.relim()
    ax.autoscale_view()
    fig.tight_layout()
    output_file = Path(output_path)
    output_file.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_file, dpi=300)
    plt.close(fig)


def _to_float_sequence(values: Iterable[float | int | str]) -> List[float]:
    """Convert iterable values to floats, passing through numbers."""
    floats: List[float] = []
    for value in values:
        if isinstance(value, (float, int)):
            floats.append(float(value))
        else:
            floats.append(float(str(value)))
    return floats


def _is_numeric_sequence(values: Iterable[float | int | str]) -> bool:
    """Return True if all values can be interpreted as numbers."""
    try:
        _to_float_sequence(values)
        return True
    except ValueError:
        return False
