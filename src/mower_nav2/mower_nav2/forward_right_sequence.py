"""Geometry for one reviewed forward-then-right trial; no ROS or CAN output."""
import copy
import math


def waypoints(pose):
    x, y = pose['xy_m']
    yaw = math.radians(pose['yaw_deg'])
    if not all(math.isfinite(v) for v in (x, y, yaw)):
        raise ValueError('Nonfinite sequence pose')
    first = [x+.6*math.cos(yaw), y+.6*math.sin(yaw)]
    turned = yaw-math.radians(35)
    second = [first[0]+.6*math.cos(turned), first[1]+.6*math.sin(turned)]
    return first, second, math.degrees(turned)


def second_leg_proposal(review, first_result):
    """Proceed only after success and a stop within the reviewed join region.

    The final goal is fixed before motion. Only the second segment's starting
    pose follows the measured first stop, within its reviewed 25 cm region.
    No failed leg may automatically retry or advance.
    """
    if (review.get('sequence_profile') != 'forward_then_right_04' or
            not review.get('motion_approved') or
            not first_result.get('completed') or first_result.get('action_status') != 4):
        raise ValueError('Require approved sequence and successful first leg')
    first, second, final_yaw = waypoints(review['pose'])
    p = first_result.get('final_pose')
    wheels = first_result.get('final_wheel_mm_s')
    if (not isinstance(p, (list, tuple)) or len(p) != 3 or
            not all(math.isfinite(v) for v in p) or
            wheels is None or len(wheels) != 4 or
            any(not math.isfinite(v) or abs(v)>10 for v in wheels) or
            first_result.get('final_api_mode') != 2):
        raise ValueError('Require acknowledged first stop and finite pose')
    if math.hypot(p[0]-first[0], p[1]-first[1]) > .25:
        raise ValueError('First stop outside reviewed join region')
    start_yaw = math.radians(review['pose']['yaw_deg'])
    delta = math.atan2(math.sin(p[2]-start_yaw), math.cos(p[2]-start_yaw))
    if abs(delta) > .15:
        raise ValueError('First stop heading outside reviewed tolerance')
    distance = math.hypot(second[0]-p[0], second[1]-p[1])
    if not .5 <= distance <= 1.:
        raise ValueError('Second leg outside short-goal limits')
    if any(abs(a-b)>1e-9 for a,b in zip(second, review['second_target_xy_m'])):
        raise ValueError('Final goal differs from reviewed sequence')
    result = copy.deepcopy(review['leg_template'])
    result.update(pose={'xy_m':p[:2], 'yaw_deg':math.degrees(p[2])},
                  proposed_target_xy_m=second, goal_yaw_deg=final_yaw,
                  distance_m=distance, motion_approved=True,
                  require_initial_heading_alignment=True)
    return result
