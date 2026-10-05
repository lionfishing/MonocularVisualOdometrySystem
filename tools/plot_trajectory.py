import argparse
import csv
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Plot a top-down monocular VO trajectory."
    )

    parser.add_argument(
        "trajectory_csv",
        type=Path,
        help="CSV file produced by project_3_vo.",
    )

    parser.add_argument(
        "output_image",
        type=Path,
        help="Output trajectory image path.",
    )

    return parser.parse_args()


def load_trajectory(csv_path):
    records = []

    with csv_path.open(
        mode="r",
        encoding="utf-8",
        newline="",
    ) as csv_file:
        reader = csv.DictReader(csv_file)

        required_columns = {
            "frame",
            "status",
            "x",
            "y",
            "z",
        }

        available_columns = set(reader.fieldnames or [])
        missing_columns = required_columns - available_columns

        if missing_columns:
            missing_text = ", ".join(
                sorted(missing_columns)
            )

            raise ValueError(
                f"Missing CSV columns: {missing_text}"
            )

        for row in reader:
            if not row["x"] or not row["z"]:
                continue

            records.append(
                {
                    "frame": row["frame"],
                    "status": row["status"],
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]),
                }
            )

    if not records:
        raise ValueError(
            "The CSV file contains no valid trajectory positions."
        )

    return records


def plot_trajectory(records, output_path):
    accepted_records = [
        record
        for record in records
        if record["status"] in {"INIT", "OK"}
    ]

    if not accepted_records:
        raise ValueError(
            "The CSV file contains no accepted poses."
        )

    skipped_records = [
        record
        for record in records
        if record["status"] == "SKIP"
    ]

    x_positions = [
        record["x"]
        for record in accepted_records
    ]

    forward_positions = [
        -record["z"]
        for record in accepted_records
    ]

    figure, axes = plt.subplots(
        figsize=(8, 8)
    )

    axes.plot(
        x_positions,
        forward_positions,
        color="tab:blue",
        marker="o",
        linewidth=2,
        markersize=5,
        label="Accepted pose",
    )

    if skipped_records:
        axes.scatter(
            [record["x"] for record in skipped_records],
            [-record["z"] for record in skipped_records],
            color="tab:red",
            marker="x",
            s=80,
            label="Skipped frame",
            zorder=4,
        )

    first_record = accepted_records[0]
    last_record = accepted_records[-1]

    axes.scatter(
        first_record["x"],
        -first_record["z"],
        color="tab:green",
        marker="s",
        s=100,
        label="Start",
        zorder=3,
    )

    axes.scatter(
        last_record["x"],
        -last_record["z"],
        color="tab:orange",
        marker="*",
        s=180,
        label="End",
        zorder=3,
    )

    for record in accepted_records:
        axes.annotate(
            record["frame"],
            (
                record["x"],
                -record["z"],
            ),
            xytext=(5, 5),
            textcoords="offset points",
            fontsize=8,
        )

    axes.set_title(
        "Simplified Monocular VO Trajectory"
    )

    axes.set_xlabel(
        "World X (arbitrary scale)"
    )

    axes.set_ylabel(
        "Forward direction, -World Z (arbitrary scale)"
    )

    axes.grid(
        visible=True,
        linestyle="--",
        alpha=0.5,
    )

    axes.set_aspect(
        "equal",
        adjustable="datalim",
    )

    axes.legend()

    output_path.parent.mkdir(
        parents=True,
        exist_ok=True,
    )

    figure.savefig(
        output_path,
        dpi=160,
        bbox_inches="tight",
    )

    plt.close(figure)


def main():
    arguments = parse_arguments()

    records = load_trajectory(
        arguments.trajectory_csv
    )

    plot_trajectory(
        records,
        arguments.output_image,
    )

    print(
        f"Trajectory plot saved to: "
        f"{arguments.output_image}"
    )


if __name__ == "__main__":
    main()