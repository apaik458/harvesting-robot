"""
Shared IK/FK core for the strawberry-harvesting arm's verification tools.

This module has no matplotlib dependency and does nothing on import besides
define constants and pure functions -- both ik_sweep_report.py (the sweep +
heatmaps + report) and ik_interactive_explorer.py (the standalone slider
window) import from here, so the actual math under test only exists in one
place, rather than each tool carrying its own copy that could silently
drift apart from the other.
"""

import numpy as np

# --- Fixed link lengths, from the firmware constants ---
kLink1LengthCm = 27.65
kLink2LengthCm = 22.35

MOTOR_MIN = 342
MOTOR_MAX = 3754

REACH_MAX = kLink1LengthCm + kLink2LengthCm
REACH_MIN = abs(kLink1LengthCm - kLink2LengthCm)

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
