"""Speedup over one thread for the #1725 parallel benchmarks on an M3 Pro.

Usage: python plot_speedup.py CORRIDOR.txt BUILDING.txt OUT_STEM
Saves OUT_STEM.png and OUT_STEM.pdf.
"""

import re
import sys

import matplotlib.pyplot as plt
import seaborn as sns

NAMES = {
    "cfs": "CFSM",
    "cfs2": "CFSM v2",
    "cfs3": "CFSM v3",
    "avm": "AVM",
    "sfm": "SFM",
    "gcfm": "GCFM",
    "warp": "WarpDriver",
}
MARKERS = ["o", "s", "D", "^", "v", "P", "X"]


def parse(path):
    """Model name -> (agents, threads, speedups) from the benchmark output."""
    runs = {}
    with open(path) as f:
        for line in f:
            head = re.match(r"(\w+)\s+(\d+) agents", line)
            if not head:
                continue
            cells = re.findall(r"(\d+):\s*[\d.]+s x\s*([\d.]+)", line)
            runs[head[1]] = (
                int(head[2]),
                [int(t) for t, _ in cells],
                [float(s) for _, s in cells],
            )
    return runs


def main():
    """Plot both benchmarks side by side.

    Saves
    -----
    OUT_STEM.png, OUT_STEM.pdf
    """
    corridor, building, stem = sys.argv[1:4]

    # --- Style Setup ---
    sns.set_theme(font_scale=1.0, style="whitegrid", font="DejaVu Sans")
    palette = sns.color_palette("colorblind", len(NAMES))

    # --- Plot ---
    fig, axes = plt.subplots(1, 2, figsize=(11, 4.6), sharey=True)
    panels = [
        (axes[0], parse(corridor), "Corridor, 300–1,500 agents, 200 it"),
        (axes[1], parse(building), "Five-storey building, 3,000–10,000 agents, 40 it"),
    ]
    for ax, runs, title in panels:
        ax.plot([1, 12], [1, 12], color="lightgrey", linestyle="--", linewidth=1)
        ax.text(9.4, 10.3, "linear", color="dimgrey", fontsize=9, rotation=33)
        ax.axvspan(6, 12.5, color="0.95", zorder=0)
        ax.text(
            9.2, 0.6, "efficiency cores", color="dimgrey", fontsize=9, ha="center"
        )
        for (key, label), color, marker in zip(NAMES.items(), palette, MARKERS):
            if key not in runs:
                continue
            agents, threads, speedup = runs[key]
            ax.plot(
                threads,
                speedup,
                marker=marker,
                color=color,
                linewidth=1.6,
                markersize=6,
                label=f"{label} ({agents:,})",
            )
        ax.set_title(title, fontsize=11, color="dimgrey", loc="left")
        ax.set_xlabel("Threads", color="dimgrey")
        ax.set_xticks([1, 2, 4, 6, 8, 12])
        ax.set_xlim(0.5, 12.5)
        ax.set_ylim(0, 12.5)
        ax.tick_params(axis="both", which="both", length=0, labelcolor="dimgrey")
        ax.legend(
            loc="upper left",
            fontsize=8.5,
            frameon=True,
            facecolor="white",
            framealpha=0.8,
            edgecolor="lightgrey",
            labelcolor="dimgrey",
        )
    axes[0].set_ylabel("Speedup over 1 thread", color="dimgrey")
    fig.suptitle(
        "PR #1725 on an Apple M3 Pro: about 5× at the 6 performance cores, "
        "flat beyond; positions identical at every thread count",
        fontsize=11.5,
        x=0.01,
        ha="left",
    )
    sns.despine(left=True, bottom=True)
    fig.tight_layout()

    # --- Save ---
    for ext in ("png", "pdf"):
        fig.savefig(f"{stem}.{ext}", dpi=150, bbox_inches="tight")


if __name__ == "__main__":
    main()
