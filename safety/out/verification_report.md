# IK Verification Sweep Report

Link lengths: kLink1LengthCm = 27.65, kLink2LengthCm = 22.35
Motor bounds: [342, 3754]
Grid: 1.0cm spacing over +/-60.0cm in x and y -> 14641 test points
Round-trip tolerance: 0.15 cm

## Results

- Out of reach (correctly rejected): 6885 points (47.03%)
- Valid: 6895 points (47.09%)
- Reachable, motor limit exceeded: 833 points (5.69%)
- Round-trip mismatch (returned motors don't reach target): 28 points (0.19%)
- BUG: unreachable but bounds check did not catch it: 0 points (0.00%)

Round-trip error over VALID points: mean=0.0389 cm, max=0.1014 cm (quantization floor is ~0.077 cm)
Round-trip error over MISMATCH points: mean=37.8557 cm, max=44.7089 cm

## Root cause check: elbow/wrist-side selection

The solve makes two decisions that must agree: which of the (up to four) elbow (x, y) candidates satisfies both link-length constraints, and the sign of `elbow_angle` (only flipped when target x < 0). For every ROUND_TRIP_MISMATCH point, every (elbow candidate, elbow_angle sign) combination was tried against forward kinematics, without modifying the function under test, to see whether a correct combination existed at all.

- 28/28 mismatches (100.0%) had a correct combination available -- the geometry was solvable, the code just paired the wrong elbow candidate with the wrong elbow_angle sign.
- 0/28 mismatches had no combination that round-tripped -- these need closer inspection, not just a branch fix.
- All 28 observed mismatches occurred at x = 0 (the y-axis), where `err_pos` and `err_neg` are exactly equal, so `abs(err_pos) < abs(err_neg)` is always false and the code always falls through to `elbow_x_neg` -- regardless of which side actually matches the (unflipped) elbow_angle sign for that target.

## Files

- `workspace_classification.png` -- categorical map of every grid point
- `roundtrip_error_map.png` -- continuous position-error heatmap for valid points

## Recommendation

1. Fix the x=0 tie: `abs(err_pos) < abs(err_neg)` is a strict inequality, so an exact tie always falls to elbow_x_neg regardless of correctness. Pick whichever (elbow candidate, elbow_angle sign) combination actually round-trips (as the diagnostic above does), instead of picking elbow_x/elbow_y independently of the elbow_angle sign.
2. Note on this report's own limitation: the Python port used here adds a NaN guard (`to_motor` returns None on NaN, forcing a (-1,-1,-1) result) that the original C++ does NOT have. In C++, converting a NaN double to int via `int(...)` is implementation-defined/undefined behavior -- it will not reliably produce a value the `< 342 || > 3754` check catches. That's why this sweep shows 0 points in the 'bounds check missed it' category: this port can't catch that failure mode by construction. Confirming actual C++ behavior for out-of-reach targets needs a test against the compiled firmware function itself, not this port.