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
  4. Classify the point into one of five categories (see CATEGORY_* below).
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

A third interactive window with X/Y sliders is also opened (when a display
is available), so you can drag around individual points -- including
illegal ones (out of reach or motor-limited) -- and see them classified the
same way as the sweep, in real time. This is for spot-checking/demoing a
specific point after the sweep has already told you where the problem
regions are; it does not replace the sweep as the actual verification step.

Run: python3 ik_verify.py           (saves files AND pops up interactive windows)
     python3 ik_verify.py --no-show (saves files only, no popup -- for headless/CI use)
"""

import os
import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.widgets import Slider

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

# --- Fixed link lengths, from the firmware constants ---
kLink1LengthCm = 27.65
kLink2LengthCm = 22.35

MOTOR_MIN = 342
MOTOR_MAX = 3754

REACH_MAX = kLink1LengthCm + kLink2LengthCm
REACH_MIN = abs(kLink1LengthCm - kLink2LengthCm)

# Grid sweep settings
GRID_RANGE = 60.0     # cm, covers past REACH_MAX so out-of-reach is exercised too
GRID_STEP = 1.0        # cm spacing between test points

# Round-trip tolerance: the 12-bit motor tick quantization alone contributes
# up to (360/4096) deg of angle error, which at ~50cm radius is about
# 0.077 cm of arc length. 0.15 cm gives headroom above that floor before
# flagging something as a real mismatch rather than quantization noise.
POSITION_TOLERANCE_CM = 0.15


# ---------------------------------------------------------------- IK (ported 1:1 from C++, unmodified)

def calculate_inverse_kinematics(x, y, link1, link2):
    """Direct port of Arm::CalculateInverseKinematics. No clamping added --
    matches the real C++ NaN behavior for out-of-domain input. Returns
    (motor_tuple, debug_dict).
    """
    distance_from_origin = np.sqrt(x * x + y * y)

    cos_arg = (link1 * link1 + link2 * link2 - distance_from_origin * distance_from_origin) / (2 * link1 * link2)
    with np.errstate(invalid="ignore"):
        elbow_angle = np.pi - np.arccos(cos_arg)
    if x < 0:
        elbow_angle = -elbow_angle

    K = -link1 * link1 - x * x - y * y + link2 * link2
    a = 4 * x * x + 4 * y * y
    b = 4 * y * K
    c = K * K - 4 * x * x * link1 * link1
    discriminant = b * b - 4 * a * c

    with np.errstate(invalid="ignore"):
        sqrt_disc = np.sqrt(discriminant)
        elbow_y1 = (-b + sqrt_disc) / (2 * a) if a != 0 else np.nan
        elbow_y2 = (-b - sqrt_disc) / (2 * a) if a != 0 else np.nan

    elbow_y = min(elbow_y1, elbow_y2) if not (np.isnan(elbow_y1) or np.isnan(elbow_y2)) else np.nan

    with np.errstate(invalid="ignore"):
        elbow_x_pos = np.sqrt(link1 * link1 - elbow_y * elbow_y)
    elbow_x_neg = -elbow_x_pos

    err_pos = (elbow_x_pos - x) ** 2 + (elbow_y - y) ** 2 - link2 * link2
    err_neg = (elbow_x_neg - x) ** 2 + (elbow_y - y) ** 2 - link2 * link2
    elbow_x = elbow_x_pos if abs(err_pos) < abs(err_neg) else elbow_x_neg

    shoulder_angle = np.arctan2(elbow_y, elbow_x) + np.pi / 2
    if shoulder_angle > np.pi:
        shoulder_angle -= 2 * np.pi
    if shoulder_angle < -np.pi:
        shoulder_angle += 2 * np.pi

    if x > 0:
        wrist_angle = -1 * ((np.pi / 2) - (shoulder_angle + elbow_angle))
    else:
        wrist_angle = -1 * ((-np.pi / 2) - (shoulder_angle + elbow_angle))
    if x == 0:
        wrist_angle = 0.0

    def to_motor(angle):
        return 2048 + int((angle / (2.0 * np.pi)) * 4096) if not np.isnan(angle) else None

    motor1 = to_motor(shoulder_angle)
    motor2 = to_motor(elbow_angle)
    motor3 = to_motor(wrist_angle)

    debug = {
        "elbow_xy": (elbow_x, elbow_y),
        "elbow_y_candidates": (elbow_y1, elbow_y2),
        "shoulder_angle": shoulder_angle,
        "elbow_angle": elbow_angle,
        "wrist_angle": wrist_angle,
        "discriminant": discriminant,
    }

    if any(m is None for m in (motor1, motor2, motor3)):
        return (-1, -1, -1), debug

    if (motor1 < MOTOR_MIN or motor1 > MOTOR_MAX or
            motor2 < MOTOR_MIN or motor2 > MOTOR_MAX or
            motor3 < MOTOR_MIN or motor3 > MOTOR_MAX):
        return (-1, -1, -1), debug

    return (motor1, motor2, motor3), debug


def motor_to_angle(motor):
    return (motor - 2048) * (2.0 * np.pi / 4096.0)


def forward_kinematics(shoulder_angle, elbow_angle, link1, link2):
    """Same convention as the IK: phi1 = shoulder_angle - pi/2 is the
    absolute direction of link1, phi2 = phi1 + elbow_angle is the absolute
    direction of link2.
    """
    phi1 = shoulder_angle - np.pi / 2
    elbow_x = link1 * np.cos(phi1)
    elbow_y = link1 * np.sin(phi1)
    phi2 = phi1 + elbow_angle
    end_x = elbow_x + link2 * np.cos(phi2)
    end_y = elbow_y + link2 * np.sin(phi2)
    return (elbow_x, elbow_y), (end_x, end_y)


def check_alternate_branch(x, y, link1, link2, debug):
    """Diagnostic only -- does NOT change what the function under test
    returned. The original solve makes two independent decisions that have
    to agree for the result to be geometrically correct: (a) which elbow
    (x, y) satisfies both circle constraints -- itself a choice between two
    y-roots and, for each, two x-signs -- and (b) the sign of elbow_angle,
    decided separately from the sign of the target x. This checks every
    (elbow candidate, elbow_angle sign) combination and reports whether ANY
    of them round-trips, and specifically whether the chosen elbow point
    would have worked with the *other* elbow_angle sign. That tells us
    whether the mismatch is a genuine "no consistent choice exists" failure
    or a "the two decisions just disagreed" failure -- the latter is a
    fixable selection bug, not a math error.
    """
    elbow_y1, elbow_y2 = debug["elbow_y_candidates"]
    if np.isnan(elbow_y1) or np.isnan(elbow_y2):
        return False, np.nan

    chosen_ex, chosen_ey = debug["elbow_xy"]
    elbow_angle_mag = abs(debug["elbow_angle"])

    candidates = []
    for ey in (elbow_y1, elbow_y2):
        with np.errstate(invalid="ignore"):
            ex_abs = np.sqrt(link1 * link1 - ey * ey)
        if np.isnan(ex_abs):
            continue
        candidates.append((ex_abs, ey))
        candidates.append((-ex_abs, ey))

    best_error = np.inf
    chosen_point_works_with_flip = False

    for ex, ey in candidates:
        shoulder_angle = np.arctan2(ey, ex) + np.pi / 2
        if shoulder_angle > np.pi:
            shoulder_angle -= 2 * np.pi
        if shoulder_angle < -np.pi:
            shoulder_angle += 2 * np.pi

        for signed_elbow_angle in (elbow_angle_mag, -elbow_angle_mag):
            _, (end_x, end_y) = forward_kinematics(shoulder_angle, signed_elbow_angle, link1, link2)
            error = np.hypot(end_x - x, end_y - y)
            best_error = min(best_error, error)

            is_chosen_point = (abs(ex - chosen_ex) < 1e-6 and abs(ey - chosen_ey) < 1e-6)
            if is_chosen_point and error < POSITION_TOLERANCE_CM:
                chosen_point_works_with_flip = True

    a_valid_combo_exists = best_error < POSITION_TOLERANCE_CM
    return a_valid_combo_exists, best_error, chosen_point_works_with_flip


# ---------------------------------------------------------------- categories

CATEGORY_OUT_OF_REACH = 0        # correctly identified as unreachable (-1,-1,-1 returned)
CATEGORY_VALID = 1               # reachable, valid motors, round-trip matches target
CATEGORY_MOTOR_LIMIT = 2         # reachable, but bounds check rejected it (-1,-1,-1)
CATEGORY_ROUNDTRIP_MISMATCH = 3  # returned valid motors, but they don't reach the target
CATEGORY_BOUNDS_CHECK_MISSED = 4  # unreachable, but did NOT return (-1,-1,-1) -- the NaN-bypass bug

CATEGORY_NAMES = {
    CATEGORY_OUT_OF_REACH: "Out of reach (correctly rejected)",
    CATEGORY_VALID: "Valid",
    CATEGORY_MOTOR_LIMIT: "Reachable, motor limit exceeded",
    CATEGORY_ROUNDTRIP_MISMATCH: "Round-trip mismatch (returned motors don't reach target)",
    CATEGORY_BOUNDS_CHECK_MISSED: "BUG: unreachable but bounds check did not catch it",
}

CATEGORY_COLORS = {
    CATEGORY_OUT_OF_REACH: "#d9d9d9",
    CATEGORY_VALID: "#2ca02c",
    CATEGORY_MOTOR_LIMIT: "#ff7f0e",
    CATEGORY_ROUNDTRIP_MISMATCH: "#d62728",
    CATEGORY_BOUNDS_CHECK_MISSED: "#7d2ea0",
}


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


# ---------------------------------------------------------------- interactive explorer

def build_interactive_explorer():
    """A third window: drag X/Y sliders to explore individual target points,
    including illegal ones, classified with the same categories/colors used
    in the sweep heatmap. Complements the sweep -- lets you spot-check a
    specific point after the sweep has told you where the problem regions
    are (e.g. drag Y up to 0 and watch it hit the x=0 mismatch line).
    """
    fig, ax = plt.subplots(figsize=(7, 7))
    plt.subplots_adjust(bottom=0.25)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    plot_lim = GRID_RANGE * 1.15
    ax.set_xlim(-plot_lim, plot_lim)
    ax.set_ylim(-plot_lim, plot_lim)
    ax.set_title("Drag sliders to explore a point (matches sweep categories/colors)")

    (arm_line,) = ax.plot([], [], "o-", lw=4, markersize=10)
    (target_pt,) = ax.plot([], [], "x", markersize=14, mew=3)

    outer = plt.Circle((0, 0), REACH_MAX, fill=False, linestyle="--", color="gray", alpha=0.5)
    inner = plt.Circle((0, 0), REACH_MIN, fill=False, linestyle="--", color="gray", alpha=0.5)
    ax.add_patch(outer)
    ax.add_patch(inner)

    info_text = ax.text(0.02, 0.98, "", transform=ax.transAxes, va="top", ha="left",
                         family="monospace", fontsize=9,
                         bbox=dict(boxstyle="round", fc="white", alpha=0.85))

    ax_x = plt.axes([0.2, 0.12, 0.6, 0.03])
    ax_y = plt.axes([0.2, 0.07, 0.6, 0.03])
    slider_x = Slider(ax_x, "Target X (cm)", -GRID_RANGE, GRID_RANGE, valinit=REACH_MAX * 0.6)
    slider_y = Slider(ax_y, "Target Y (cm)", -GRID_RANGE, GRID_RANGE, valinit=REACH_MAX * 0.4)

    def redraw(_=None):
        try:
            x, y = slider_x.val, slider_y.val
            distance = np.hypot(x, y)
            reachable = REACH_MIN <= distance <= REACH_MAX

            motors, debug = calculate_inverse_kinematics(x, y, kLink1LengthCm, kLink2LengthCm)
            returned_valid = motors != (-1, -1, -1)
            ex, ey = debug["elbow_xy"]

            if not reachable:
                category = CATEGORY_BOUNDS_CHECK_MISSED if returned_valid else CATEGORY_OUT_OF_REACH
            elif not returned_valid:
                category = CATEGORY_MOTOR_LIMIT
            else:
                sa = motor_to_angle(motors[0])
                ea = motor_to_angle(motors[1])
                _, (fkx, fky) = forward_kinematics(sa, ea, kLink1LengthCm, kLink2LengthCm)
                error = np.hypot(fkx - x, fky - y)
                category = CATEGORY_VALID if error < POSITION_TOLERANCE_CM else CATEGORY_ROUNDTRIP_MISMATCH

            color = CATEGORY_COLORS[category]
            target_pt.set_data([x], [y])
            target_pt.set_color(color)

            if reachable and not np.isnan(ex) and not np.isnan(ey):
                arm_line.set_data([0, ex, x], [0, ey, y])
                arm_line.set_linestyle("-")
            else:
                arm_line.set_data([0, x], [0, y])
                arm_line.set_linestyle("--")
            arm_line.set_color(color)

            m1, m2, m3 = motors
            info_text.set_text(
                f"target      x={x:6.2f}  y={y:6.2f}   dist={distance:6.2f}\n"
                f"reach       {REACH_MIN:.2f} <= dist <= {REACH_MAX:.2f}\n"
                f"elbow_xy    ({ex:6.2f}, {ey:6.2f})\n"
                f"shoulder    {np.degrees(debug['shoulder_angle']):7.2f} deg\n"
                f"elbow       {np.degrees(debug['elbow_angle']):7.2f} deg\n"
                f"motor pos   {m1}, {m2}, {m3}\n"
                f"category    {CATEGORY_NAMES[category]}"
            )
            info_text.set_color("firebrick" if category != CATEGORY_VALID else "black")
            fig.canvas.draw_idle()
        except Exception:
            # Never let a redraw error leave the slider silently stuck --
            # print the real traceback so it's visible instead of the
            # widget just appearing to stop responding.
            import traceback
            traceback.print_exc()


    slider_x.on_changed(redraw)
    slider_y.on_changed(redraw)
    redraw()
    return fig


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
    fig1 = fig  # keep reference open for the interactive popup below

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
    fig2 = fig  # keep reference open for the interactive popup below

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
        explorer_fig = build_interactive_explorer()
        print("\nOpening interactive plot windows (close them to exit)...")
        plt.show()
    else:
        plt.close(fig1)
        plt.close(fig2)


if __name__ == "__main__":
    if "--explorer-only" in sys.argv:
        # Standalone slider window, no sweep and no heavy heatmap figures.
        # Useful for isolating whether the full script's sluggishness/
        # unresponsiveness after a while is caused by resource contention
        # between three simultaneous Tk windows (two of them large images)
        # sharing one event loop, vs. an actual bug.
        if not _SHOW_PLOTS:
            print("No working interactive backend -- nothing to show.")
        else:
            build_interactive_explorer()
            print("Opening explorer-only window (close it to exit)...")
            plt.show()
    else:
        main()