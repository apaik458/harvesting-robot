"""
Standalone interactive IK explorer for the strawberry-harvesting arm.

Drag the X/Y sliders to move a target point around, including illegal ones
(out of reach or motor-limited), and see it classified live with the same
categories/colors ik_sweep_report.py's heatmap uses. This is for spot-
checking/demoing a specific point after the sweep has already told you
where the problem regions are -- it does not replace the sweep as the
actual verification step (run ik_sweep_report.py for that).

This used to be a third window opened by the same script as the sweep and
its two heatmap figures. Sharing one Tk/Qt event loop with two other large-
image windows meant mouse-drag events on the sliders could queue up faster
than the shared loop could drain them, so dragging appeared to freeze after
a few seconds. Running this as its own process gives it its own dedicated
event loop with nothing else competing for it.

Run: python3 ik_interactive_explorer.py           (pops up the window)
     python3 ik_interactive_explorer.py --no-show (nothing to do -- this
                                                    script has no file
                                                    output; use
                                                    ik_sweep_report.py)
"""

import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

from ik_verification_core import (
    kLink1LengthCm, kLink2LengthCm, REACH_MAX, REACH_MIN, POSITION_TOLERANCE_CM,
    calculate_inverse_kinematics, motor_to_angle, forward_kinematics,
    CATEGORY_OUT_OF_REACH, CATEGORY_VALID, CATEGORY_MOTOR_LIMIT,
    CATEGORY_ROUNDTRIP_MISMATCH, CATEGORY_BOUNDS_CHECK_MISSED,
    CATEGORY_NAMES, CATEGORY_COLORS,
)

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
        print("Nothing to show -- this script has no non-interactive output. Run ik_sweep_report.py instead.")
        _SHOW_PLOTS = False

# Match the sweep's plotted range so the two tools look consistent.
GRID_RANGE = 60.0


def build_interactive_explorer():
    """Drag X/Y sliders to explore individual target points, including
    illegal ones, classified with the same categories/colors used in the
    sweep heatmap. Complements the sweep -- lets you spot-check a specific
    point after the sweep has told you where the problem regions are (e.g.
    drag Y up to 0 and watch it hit the x=0 mismatch line).
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
    # valstep coarsens the drag granularity so a continuous drag fires far
    # fewer on_changed events (each one triggers a full recompute + redraw).
    # Without it, every sub-pixel mouse movement is a new event.
    slider_x = Slider(ax_x, "Target X (cm)", -GRID_RANGE, GRID_RANGE, valinit=REACH_MAX * 0.6, valstep=0.5)
    slider_y = Slider(ax_y, "Target Y (cm)", -GRID_RANGE, GRID_RANGE, valinit=REACH_MAX * 0.4, valstep=0.5)

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


if __name__ == "__main__":
    if not _SHOW_PLOTS:
        print("No working interactive backend -- nothing to show.")
    else:
        build_interactive_explorer()
        print("Opening interactive explorer window (close it to exit)...")
        plt.show()
