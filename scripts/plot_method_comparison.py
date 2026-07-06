"""Compare the success curves of two or more training runs on the same task.

Reads the TensorBoard scalar logs of each run and overlays their success-rate
curves, so different learning methods (e.g. residual vs. from-scratch PPO) can
be compared directly on one figure for a report.

Each ``--run`` may point at a single run *or* at several seed runs (via a shell
glob or a comma-separated list); with several seeds the mean curve is drawn with
a shaded ``±1 std`` error band, so the comparison shows reproducibility rather
than a single lucky seed.

Example::

    # one run per method
    ./.conda-env/bin/python scripts/plot_method_comparison.py \
        --run "Residual (Expert + Korrektur)=runs/demo_coop_random" \
        --run "PPO von Null (ohne Expert)=runs/demo_scratch" \
        --output figures/comparison --expert-baseline 58

    # several seeds per method -> mean ±std bands (quote the glob so the script expands it)
    ./.conda-env/bin/python scripts/plot_method_comparison.py \
        --run "Residual=runs/residual_seed*" \
        --run "PPO von Null=runs/scratch_seed*" \
        --output figures/comparison --expert-baseline 58
"""

from __future__ import annotations

import argparse
import glob as globlib
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

try:
    from plot_training_progress import (
        aggregate,
        collect_seeds,
        draw,
        first_present,
        matching,
        resolve_run_dir,
    )
except ModuleNotFoundError:
    from scripts.plot_training_progress import (
        aggregate,
        collect_seeds,
        draw,
        first_present,
        matching,
        resolve_run_dir,
    )


def expand_run_dirs(pattern: str) -> list[Path]:
    """Turn one --run path (glob and/or comma-separated) into resolved run dirs."""
    parts = [part for part in pattern.split(",") if part]
    paths: list[str] = []
    for part in parts:
        matches = sorted(globlib.glob(part))
        paths.extend(matches if matches else [part])
    if not paths:
        raise SystemExit(f"No run directories matched: {pattern}")
    return [resolve_run_dir(Path(path)) for path in paths]


def success_tag(tags, metric):
    """Pick the success-rate tag for the requested metric (eval / rolling)."""
    eval_tag = matching(tags, "eval_current_", "/success_rate")
    rolling_tag = first_present(tags, "box_push/success_rate_100")
    tag = eval_tag if metric == "eval" else rolling_tag
    return tag or eval_tag or rolling_tag


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare success curves of multiple training runs.")
    parser.add_argument(
        "--run",
        action="append",
        required=True,
        metavar="LABEL=LOGDIR",
        help="A run to plot, as LABEL=path/to/logdir. The path may be a glob or a comma-separated "
        "list of seed runs (mean ±std band). Repeatable (give it at least twice).",
    )
    parser.add_argument("--output", type=Path, default=Path("figures/comparison"))
    parser.add_argument(
        "--metric",
        choices=("rolling", "eval"),
        default="rolling",
        help="rolling = box_push/success_rate_100 (smooth training average); eval = deterministic eval_current.",
    )
    parser.add_argument(
        "--no-band",
        action="store_true",
        help="Plot only the mean line per method, without the shaded ±std band, even with several seeds.",
    )
    parser.add_argument("--expert-baseline", type=float, default=None, help="Draw a reference line at this success %%.")
    parser.add_argument("--format", choices=("png", "pdf"), default="png")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("--title", default="Vergleich der Lernmethoden: Erfolgsrate (coop_random)")
    args = parser.parse_args()

    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    if args.expert_baseline is not None:
        ax.axhline(
            args.expert_baseline,
            ls="--",
            color="gray",
            linewidth=1.2,
            label=f"Expert-Start ({args.expert_baseline:.0f} %)",
        )

    colors = ["tab:blue", "tab:red", "tab:green", "tab:purple"]
    band = not args.no_band
    plotted_any = False
    for index, spec in enumerate(args.run):
        if "=" not in spec:
            raise SystemExit(f"--run must be LABEL=LOGDIR, got: {spec}")
        label, pattern = spec.split("=", 1)
        run_dirs = expand_run_dirs(pattern)
        seeds, tags = collect_seeds(run_dirs)
        tag = success_tag(tags, args.metric)
        if tag is None:
            print(f"  WARN: no success metric found in {pattern}, skipping")
            continue
        agg = aggregate(seeds, tag, 100.0)
        if agg is None:
            print(f"  WARN: success metric empty in {pattern}, skipping")
            continue
        draw(ax, agg, label, colors[index % len(colors)], band, marker="o", markersize=3)
        plotted_any = True
        _, mean, _, n_seeds = agg
        print(f"  {label}: {len(run_dirs)} seed(s), {len(mean)} points, "
              f"final={mean[-1]:.0f}%, mean={mean.mean():.0f}%")

    if not plotted_any:
        raise SystemExit("No runs had a success metric to plot.")

    ax.set_title(args.title)
    ax.set_xlabel("Trainings-Timesteps")
    ax.set_ylabel("Erfolgsrate [%]")
    ax.set_ylim(-2, 102)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)
    fig.tight_layout()

    args.output.mkdir(parents=True, exist_ok=True)
    out = args.output / f"method_comparison.{args.format}"
    fig.savefig(out, dpi=args.dpi)
    print(f"saved {out}")
    plt.close(fig)


if __name__ == "__main__":
    main()
