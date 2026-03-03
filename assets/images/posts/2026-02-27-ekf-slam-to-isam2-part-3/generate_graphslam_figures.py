import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

# How to run: from repo root execute
# `python3 assets/images/posts/2026-02-27-ekf-slam-to-isam2-part-3/generate_graphslam_figures.py`

OUTPUT_DIR = Path(__file__).resolve().parent

PALETTE = {
    "bg": "#f6fbfa",
    "text": "#12343b",
    "axis": "#2f7e79",
    "curve": "#58b8a7",
    "point": "#e9785d",
    "var_face": "#64c2b4",
    "var_edge": "#2f7e79",
    "fac_face": "#1f5d5a",
    "fac_edge": "#12343b",
    "edge": "#3f8f88",
}


def style():
    plt.rcParams.update(
        {
            "figure.facecolor": PALETTE["bg"],
            "axes.facecolor": PALETTE["bg"],
            "savefig.facecolor": PALETTE["bg"],
            "font.family": "DejaVu Sans",
            "text.color": PALETTE["text"],
            "axes.labelcolor": PALETTE["text"],
            "axes.edgecolor": PALETTE["axis"],
            "xtick.color": PALETTE["axis"],
            "ytick.color": PALETTE["axis"],
        }
    )


def save_figure(fig, stem, image_format):
    if image_format in {"png", "both"}:
        png_path = OUTPUT_DIR / f"{stem}.png"
        fig.savefig(png_path, dpi=220, bbox_inches="tight")
    if image_format in {"svg", "both"}:
        svg_path = OUTPUT_DIR / f"{stem}.svg"
        fig.savefig(svg_path, bbox_inches="tight")
    plt.close(fig)


def add_node(ax, xy, label, kind="variable", size=0.3):
    x_pos, y_pos = xy
    if kind == "variable":
        patch = Circle(
            (x_pos, y_pos),
            size,
            facecolor=PALETTE["var_face"],
            edgecolor=PALETTE["var_edge"],
            linewidth=2,
            zorder=2,
        )
    else:
        patch = Rectangle(
            (x_pos - size, y_pos - size),
            2 * size,
            2 * size,
            facecolor=PALETTE["fac_face"],
            edgecolor=PALETTE["fac_edge"],
            linewidth=2,
            joinstyle="round",
            zorder=2,
        )
    ax.add_patch(patch)
    ax.text(
        x_pos,
        y_pos,
        label,
        ha="center",
        va="center",
        color="white",
        fontsize=11,
        weight="bold",
        zorder=3,
    )


def connect(ax, start, end):
    ax.plot(
        [start[0], end[0]],
        [start[1], end[1]],
        color=PALETTE["edge"],
        linewidth=2.6,
        solid_capstyle="round",
        zorder=1,
    )


def draw_distribution_vs_point_estimate(image_format):
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.3), constrained_layout=True)

    x_values = [i / 100.0 for i in range(0, 601)]
    y_values = [
        0.72 * gaussian(x, 1.8, 0.42) + 0.46 * gaussian(x, 3.95, 0.58) for x in x_values
    ]

    ax = axes[0]
    ax.plot(x_values, y_values, color=PALETTE["curve"], linewidth=3)
    ax.fill_between(x_values, y_values, color=PALETTE["curve"], alpha=0.22)
    ax.set_xlim(0, 6)
    ax.set_ylim(0, max(y_values) * 1.15)
    ax.set_title("Full distribution view", fontsize=18, weight="bold")
    ax.set_xlabel("state x", fontsize=15)
    ax.set_ylabel("probability", fontsize=15)
    ax.tick_params(labelsize=13)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    ax = axes[1]
    ax.plot(x_values, y_values, color=PALETTE["curve"], linewidth=2, alpha=0.25)
    x_star = 1.84
    y_star = 0.72 * gaussian(x_star, 1.8, 0.42) + 0.46 * gaussian(x_star, 3.95, 0.58)
    ax.scatter([x_star], [y_star], s=120, color=PALETTE["point"], zorder=3)
    ax.vlines(x_star, 0, y_star, colors=PALETTE["point"], linestyles="--", linewidth=1.8)
    ax.annotate(
        "choose one x*",
        xy=(x_star, y_star),
        xytext=(3.0, y_star * 0.85),
        color=PALETTE["text"],
        arrowprops={"arrowstyle": "->", "color": PALETTE["point"], "lw": 1.8},
        fontsize=14,
    )
    ax.set_xlim(0, 6)
    ax.set_ylim(0, max(y_values) * 1.15)
    ax.set_title("Point-estimate view", fontsize=18, weight="bold")
    ax.set_xlabel("state x", fontsize=15)
    ax.tick_params(labelsize=13)
    ax.set_yticklabels([])
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    save_figure(fig, "distribution_vs_point_estimate", image_format)


def gaussian(x_val, mean, std):
    norm = std * (2.0 * 3.141592653589793) ** 0.5
    z_val = (x_val - mean) / std
    return (2.718281828459045 ** (-0.5 * z_val * z_val)) / norm


def draw_factor_graph_hello_world(image_format):
    fig, ax = plt.subplots(figsize=(7.2, 1.3))
    ax.set_xlim(0.7, 6.6)
    ax.set_ylim(1.0, 2.0)
    ax.set_aspect("equal", adjustable="box")
    ax.axis("off")

    f1 = (1.1, 1.5)
    x1 = (2.8, 1.5)
    f12 = (4.5, 1.5)
    x2 = (6.2, 1.5)

    connect(ax, f1, x1)
    connect(ax, x1, f12)
    connect(ax, f12, x2)

    add_node(ax, f1, "f1", kind="factor")
    add_node(ax, x1, "x1", kind="variable")
    add_node(ax, f12, "f12", kind="factor")
    add_node(ax, x2, "x2", kind="variable")

    save_figure(fig, "factor_graph_hello_world", image_format)


def draw_factor_graph_tiny_slam(image_format):
    fig, ax = plt.subplots(figsize=(7.2, 4.8), constrained_layout=True)
    ax.set_xlim(0.9, 8.3)
    ax.set_ylim(0.2, 5.6)
    ax.axis("off")

    f_prior = (2.0, 4.8)
    x0 = (2.0, 3.5)
    f_motion = (4.6, 3.5)
    x1 = (7.2, 3.5)
    f_meas0 = (3.4, 1.9)
    f_meas1 = (5.8, 1.9)
    m1 = (4.6, 0.9)

    for start, end in [
        (f_prior, x0),
        (x0, f_motion),
        (f_motion, x1),
        (x0, f_meas0),
        (x1, f_meas1),
        (f_meas0, m1),
        (f_meas1, m1),
    ]:
        connect(ax, start, end)

    add_node(ax, f_prior, "prior", kind="factor")
    add_node(ax, x0, "x0", kind="variable")
    add_node(ax, f_motion, "motion", kind="factor")
    add_node(ax, x1, "x1", kind="variable")
    add_node(ax, f_meas0, "meas0", kind="factor")
    add_node(ax, f_meas1, "meas1", kind="factor")
    add_node(ax, m1, "m1", kind="variable")

    ax.text(5, 5.55, "Tiny Full-SLAM factor graph", ha="center", fontsize=14, weight="bold", color=PALETTE["text"])

    save_figure(fig, "factor_graph_tiny_full_slam", image_format)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate GraphSLAM post figures."
    )
    parser.add_argument(
        "--format",
        choices=["svg", "png", "both"],
        default="svg",
        help="Image format to generate (default: svg).",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    style()
    draw_distribution_vs_point_estimate(args.format)
    draw_factor_graph_hello_world(args.format)
    draw_factor_graph_tiny_slam(args.format)
    print(f"Saved {args.format} figures to: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
