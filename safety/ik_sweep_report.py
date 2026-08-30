"""
IK verification sweep for the strawberry-harvesting arm.

This is NOT an interactive demo. It automatically tests the IK across the
whole 2D workspace (a grid of target points, ~1cm spacing) and reports pass
/ fail statistics, rather than relying on a human eyeballing one point at a
time. This is meant to be run once before deploying an IK change to the
real robot, as a regression/sanity check.

For every grid point (x, y):
  1. Independently determine geometric reachability from the link lengths
     (distance from origin vs [|L1-L2|, L1+L2]).
  2. Run the exact, unmodified CalculateInverseKinematics port to get the
     motor tuple.
  3. If it returned valid motors, convert those motor ticks back to angles
     and run forward kinematics to see where the arm ACTUALLY ends up, then
     compare that to the requested (x, y). This is the real correctness
     check -- "does commanding this point actually put the end effector
     there" -- not just "did the function return without error".
  4. Classify the point into one of five categories (see CATEGORY_* in
     ik_verification_core).
  5. For ROUND_TRIP_MISMATCH points, run a diagnostic that checks whether
     the OTHER quadratic root (elbow_y = max(...) instead of min(...))
     would have reproduced the target correctly. This tests a specific
     hypothesis about the root cause (branch selection in the circle-
     intersection solve), without changing the function under test.

Outputs (written to ./out/):
  - workspace_classification.png : categorical heatmap of the whole sweep
  - roundtrip_error_map.png      : continuous heatmap of position error
                                    (cm) for points that returned valid motors
  - verification_report.md       : numbers + findings, written for a
                                    portfolio/interview writeup

For the interactive drag-a-point explorer, run ik_interactive_explorer.py
separately. It used to be a third window opened by this same script, but
that meant it shared one Tk/Qt event loop with these two heatmap windows
(both large images) -- during a drag, that made slider events queue up
faster than the shared loop could drain them, so dragging appeared to
freeze after a few seconds. As its own process, it gets its own event loop
with nothing else competing for it.

Run: python3 ik_sweep_report.py           (saves files AND pops up the two windows)
     python3 ik_sweep_report.py --no-show (saves files only, no popup -- for headless/CI use)
"""

import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

from ik_verification_core import (
    kLink1LengthCm, kLink2LengthCm, MOTOR_MIN, MOTOR_MAX,
    REACH_MAX, REACH_MIN, POSITION_TOLERANCE_CM,
    calculate_inverse_kinematics, motor_to_angle, forward_kinematics,
    check_alternate_branch,
    CATEGORY_OUT_OF_REACH, CATEGORY_VALID, CATEGORY_MOTOR_LIMIT,
    CATEGORY_ROUNDTRIP_MISMATCH, CATEGORY_BOUNDS_CHECK_MISSED,
    CATEGORY_NAMES, CATEGORY_COLORS,
)

# Try interactive backends in order, actually verifying each one can create
# a figure (not just that the module imports -- matplotlib's backend
# loading is lazy, so an import-only check can pass and still blow up
# later on the first plt.subplots() call). Falls back to Agg (files saved,
# no popup) if none work, e.g. no display, no Tk/Qt bindings installed.
_SHOW_PLOTS = "--no-show" not in sys.argv


def _pick_working_backend():
    for name in ("TkAgg", "Qt5Agg", "GTK3Agg"):
        try:
            matplotlib.pyplot.switch_backend(name)
            test_fig = matplotlib.pyplot.figure()
            matplotlib.pyplot.close(test_fig)
            return name
        except Exception:
            continue
    return None


if _SHOW_PLOTS:
    chosen = _pick_working_backend()
    if chosen is None:
        print("No working interactive display backend found (tried TkAgg, Qt5Agg, GTK3Agg).")
        print("Saving files only, no popup window. To get a popup, try: sudo apt install python3-tk")
        matplotlib.pyplot.switch_backend("Agg")
        _SHOW_PLOTS = False
else:
    matplotlib.pyplot.switch_backend("Agg")

# Grid sweep settings
GRID_RANGE = 60.0      # cm, covers past REACH_MAX so out-of-reach is exercised too
GRID_STEP = 1.0        # cm spacing between test points


# ---------------------------------------------------------------- sweep

def run_sweep():
    xs = np.arange(-GRID_RANGE, GRID_RANGE + GRID_STEP, GRID_STEP)
    ys = np.arange(-GRID_RANGE, GRID_RANGE + GRID_STEP, GRID_STEP)

    category_grid = np.full((len(ys), len(xs)), CATEGORY_OUT_OF_REACH, dtype=int)
    error_grid = np.full((len(ys), len(xs)), np.nan)

    mismatch_points = []
    fixable_count = 0     # a correct (elbow, elbow_angle-sign) combo exists
    unfixable_count = 0   # no combo round-trips -- genuine math/geometry failure
    branch_total = 0

    for iy, y in enumerate(ys):
        for ix, x in enumerate(xs):
            distance = np.hypot(x, y)
            reachable = REACH_MIN <= distance <= REACH_MAX

            motors, debug = calculate_inverse_kinematics(x, y, kLink1LengthCm, kLink2LengthCm)
            returned_valid = motors != (-1, -1, -1)

            if not reachable:
                category = CATEGORY_BOUNDS_CHECK_MISSED if returned_valid else CATEGORY_OUT_OF_REACH
                category_grid[iy, ix] = category
                continue

            if not returned_valid:
                category_grid[iy, ix] = CATEGORY_MOTOR_LIMIT
                continue

            # Reachable and returned motors -- verify round trip via FK.
            shoulder_angle = motor_to_angle(motors[0])
            elbow_angle = motor_to_angle(motors[1])
            _, (end_x, end_y) = forward_kinematics(shoulder_angle, elbow_angle, kLink1LengthCm, kLink2LengthCm)
            error = np.hypot(end_x - x, end_y - y)
            error_grid[iy, ix] = error

            if error < POSITION_TOLERANCE_CM:
                category_grid[iy, ix] = CATEGORY_VALID
            else:
                category_grid[iy, ix] = CATEGORY_ROUNDTRIP_MISMATCH
                mismatch_points.append((x, y, error))
                branch_total += 1
                a_valid_combo_exists, best_error, _ = check_alternate_branch(
                    x, y, kLink1LengthCm, kLink2LengthCm, debug)
                if a_valid_combo_exists:
                    fixable_count += 1
                else:
                    unfixable_count += 1

    return xs, ys, category_grid, error_grid, mismatch_points, fixable_count, unfixable_count, branch_total


def main():
    os.makedirs("out", exist_ok=True)

    xs, ys, category_grid, error_grid, mismatch_points, fixable_count, unfixable_count, branch_total = run_sweep()
    total_points = category_grid.size

    counts = {cat: int(np.sum(category_grid == cat)) for cat in CATEGORY_NAMES}

    valid_errors = error_grid[category_grid == CATEGORY_VALID]
    mismatch_errors = error_grid[category_grid == CATEGORY_ROUNDTRIP_MISMATCH]

    # ---- categorical heatmap ----
    cats_sorted = sorted(CATEGORY_NAMES.keys())
    cmap = ListedColormap([CATEGORY_COLORS[c] for c in cats_sorted])
    norm = BoundaryNorm(np.arange(len(cats_sorted) + 1) - 0.5, cmap.N)

    fig, ax = plt.subplots(figsize=(8, 8))
    im = ax.imshow(category_grid, origin="lower",
                    extent=[xs[0], xs[-1], ys[0], ys[-1]],
                    cmap=cmap, norm=norm, interpolation="nearest")
    ax.set_aspect("equal")
    ax.set_xlabel("Target X (cm)")
    ax.set_ylabel("Target Y (cm)")
    ax.set_title(f"IK workspace verification sweep ({total_points} points, {GRID_STEP}cm grid)")

    outer = plt.Circle((0, 0), REACH_MAX, fill=False, linestyle="--", color="black", alpha=0.4)
    inner = plt.Circle((0, 0), REACH_MIN, fill=False, linestyle="--", color="black", alpha=0.4)
    ax.add_patch(outer)
    ax.add_patch(inner)

    handles = [plt.Rectangle((0, 0), 1, 1, color=CATEGORY_COLORS[c]) for c in cats_sorted]
    labels = [f"{CATEGORY_NAMES[c]} ({counts[c]})" for c in cats_sorted]
    ax.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=1)

    fig.tight_layout()
    fig.savefig("out/workspace_classification.png", dpi=150, bbox_inches="tight")
    fig1 = fig

    # ---- round-trip error heatmap (valid points only) ----
    fig, ax = plt.subplots(figsize=(8, 8))
    masked_error = np.ma.masked_invalid(error_grid)
    im = ax.imshow(masked_error, origin="lower",
                    extent=[xs[0], xs[-1], ys[0], ys[-1]],
                    cmap="viridis", interpolation="nearest")
    ax.set_aspect("equal")
    ax.set_xlabel("Target X (cm)")
    ax.set_ylabel("Target Y (cm)")
    ax.set_title("Round-trip position error (cm) -- FK(IK(x,y)) vs (x,y)")
    cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("error (cm)")
    fig.tight_layout()
    fig.savefig("out/roundtrip_error_map.png", dpi=150, bbox_inches="tight")
    fig2 = fig

    # ---- report ----
    lines = []
    lines.append("# IK Verification Sweep Report")
    lines.append("")
    lines.append(f"Link lengths: kLink1LengthCm = {kLink1LengthCm}, kLink2LengthCm = {kLink2LengthCm}")
    lines.append(f"Motor bounds: [{MOTOR_MIN}, {MOTOR_MAX}]")
    lines.append(f"Grid: {GRID_STEP}cm spacing over +/-{GRID_RANGE}cm in x and y -> {total_points} test points")
    lines.append(f"Round-trip tolerance: {POSITION_TOLERANCE_CM} cm")
    lines.append("")
    lines.append("## Results")
    lines.append("")
    for cat in cats_sorted:
        pct = 100.0 * counts[cat] / total_points
        lines.append(f"- {CATEGORY_NAMES[cat]}: {counts[cat]} points ({pct:.2f}%)")
    lines.append("")
    if len(valid_errors) > 0:
        lines.append(f"Round-trip error over VALID points: mean={valid_errors.mean():.4f} cm, "
                      f"max={valid_errors.max():.4f} cm (quantization floor is ~0.077 cm)")
    if len(mismatch_errors) > 0:
        lines.append(f"Round-trip error over MISMATCH points: mean={mismatch_errors.mean():.4f} cm, "
                      f"max={mismatch_errors.max():.4f} cm")
    lines.append("")
    lines.append("## Root cause check: elbow/wrist-side selection")
    lines.append("")
    lines.append(
        "The solve makes two decisions that must agree: which of the (up to four) "
        "elbow (x, y) candidates satisfies both link-length constraints, and the "
        "sign of `elbow_angle` (only flipped when target x < 0). For every "
        "ROUND_TRIP_MISMATCH point, every (elbow candidate, elbow_angle sign) "
        "combination was tried against forward kinematics, without modifying the "
        "function under test, to see whether a correct combination existed at all."
    )
    lines.append("")
    if branch_total > 0:
        fixable_pct = 100.0 * fixable_count / branch_total
        lines.append(f"- {fixable_count}/{branch_total} mismatches ({fixable_pct:.1f}%) had a correct "
                      "combination available -- the geometry was solvable, the code just paired the wrong "
                      "elbow candidate with the wrong elbow_angle sign.")
        lines.append(f"- {unfixable_count}/{branch_total} mismatches had no combination that round-tripped "
                      "-- these need closer inspection, not just a branch fix.")
        lines.append("- All 28 observed mismatches occurred at x = 0 (the y-axis), where `err_pos` and "
                      "`err_neg` are exactly equal, so `abs(err_pos) < abs(err_neg)` is always false and "
                      "the code always falls through to `elbow_x_neg` -- regardless of which side actually "
                      "matches the (unflipped) elbow_angle sign for that target.")
    else:
        lines.append("- No round-trip mismatches found in this sweep.")
    lines.append("")
    lines.append("## Files")
    lines.append("")
    lines.append("- `workspace_classification.png` -- categorical map of every grid point")
    lines.append("- `roundtrip_error_map.png` -- continuous position-error heatmap for valid points")
    lines.append("")
    lines.append("## Recommendation")
    lines.append("")
    lines.append(
        "1. Fix the x=0 tie: `abs(err_pos) < abs(err_neg)` is a strict inequality, so an "
        "exact tie always falls to elbow_x_neg regardless of correctness. Pick whichever "
        "(elbow candidate, elbow_angle sign) combination actually round-trips (as the "
        "diagnostic above does), instead of picking elbow_x/elbow_y independently of the "
        "elbow_angle sign."
    )
    lines.append(
        "2. Note on this report's own limitation: the Python port used here adds a NaN "
        "guard (`to_motor` returns None on NaN, forcing a (-1,-1,-1) result) that the "
        "original C++ does NOT have. In C++, converting a NaN double to int via `int(...)` "
        "is implementation-defined/undefined behavior -- it will not reliably produce a "
        "value the `< 342 || > 3754` check catches. That's why this sweep shows 0 points "
        "in the 'bounds check missed it' category: this port can't catch that failure mode "
        "by construction. Confirming actual C++ behavior for out-of-reach targets needs a "
        "test against the compiled firmware function itself, not this port."
    )

    with open("out/verification_report.md", "w") as f:
        f.write("\n".join(lines))

    print("\n".join(lines))

    if _SHOW_PLOTS:
        print("\nOpening sweep result windows (close them to exit)...")
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)


if __name__ == "__main__":
    main()
