"""Pure geometry and budgets for one reviewed U-route, with no hardware I/O."""
import math


def path_length(path):
    return sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(path,path[1:]))


def validate_review(proposal, require_physical_confirmation=True):
    profiles = {'static_keepout_right_u_04': (.4, 8., 14., 150., 16.),
                'static_keepout_gentle_u_03': (.3, 8., 15., 150., 16.),
                'static_keepout_gentle_arc_03': (.3, 2.5, 4., 45., 5.),
                'static_obstacle_wait_straight_03': (.3, 3., 4., 75., 6.)}
    if proposal.get('route_profile') not in profiles:
        raise ValueError('Unsupported reviewed route')
    linear_limit, min_length, path_limit, time_limit, travel_limit = profiles[proposal['route_profile']]
    expected = dict(max_linear_speed_mps=linear_limit, max_angular_speed_radps=.2,
                    corridor_half_width_m=.2, max_run_seconds=time_limit,
                    max_travel_m=travel_limit, review_radius_m=.98)
    if any(proposal.get(k)!=v for k,v in expected.items()):
        raise ValueError('Unsupported route budgets')
    if require_physical_confirmation and not proposal.get('physical_whole_route_confirmed'):
        raise ValueError('Whole route physical review required')
    reference=proposal['path_xy_yaw_rad']
    execution=proposal.get('execution_path_xy_yaw_rad',reference)
    for path in (reference,execution):
        if not 2<=len(path)<=1000 or any(len(p)!=3 or not all(math.isfinite(v) for v in p) for p in path):
            raise ValueError('Invalid path geometry')
        if not min_length<=path_length(path)<=path_limit:
            raise ValueError('Route length outside the reviewed scope')
    pose=proposal['pose'];goal=proposal['goal_xy_m']
    if (len(pose['xy_m'])!=2 or len(goal)!=2 or
            not all(math.isfinite(v) for v in (*pose['xy_m'],pose['yaw_deg'],*goal,proposal['goal_yaw_deg']))):
        raise ValueError('Invalid start or goal')
    if math.dist(reference[0][:2],pose['xy_m'])>.2 or math.dist(reference[-1][:2],goal)>.05:
        raise ValueError('Reference does not match reviewed start and goal')
    if math.dist(execution[0][:2],reference[0][:2])>.05 or math.dist(execution[-1][:2],goal)>.05:
        raise ValueError('Execution endpoints changed')
    if proposal['route_profile'] == 'static_obstacle_wait_straight_03':
        if proposal.get('obstacle_wait_policy') != {
                'max_wait_seconds': 30., 'minimum_wait_seconds': 5.,
                'clear_seconds': 2., 'stationary_seconds': 1., 'maximum_resumes': 1}:
            raise ValueError('Unsupported obstacle wait scope')
        heading = math.radians(pose['yaw_deg'])
        for path in (reference, execution):
            previous = -.001
            for x, y, yaw in path:
                dx, dy = x-pose['xy_m'][0], y-pose['xy_m'][1]
                along = dx*math.cos(heading)+dy*math.sin(heading)
                lateral = -dx*math.sin(heading)+dy*math.cos(heading)
                if (abs(lateral) > .02 or along < previous-.001 or
                        abs(math.atan2(math.sin(yaw-heading), math.cos(yaw-heading))) > .02):
                    raise ValueError('Wait trial requires a straight forward route')
                previous = along
    # Interpolate each segment: endpoints alone cannot prove containment.
    for a,b in zip(execution,execution[1:]):
        length=math.dist(a[:2],b[:2]);count=max(1,math.ceil(length/.02))
        for i in range(count+1):
            xy=[a[j]+(b[j]-a[j])*i/count for j in (0,1)]
            if project(reference,xy)[0]>.1:
                raise ValueError('Execution path leaves reference tube')
    return reference, execution


def project(path, xy):
    if len(xy)!=2 or not all(math.isfinite(v) for v in xy):
        raise ValueError('Invalid robot position')
    best=(math.inf,0.);offset=0.
    for a,b in zip(path,path[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<=1e-9:continue
        t=max(0.,min(1.,((xy[0]-a[0])*dx+(xy[1]-a[1])*dy)/(length*length)))
        distance=math.hypot(xy[0]-a[0]-t*dx,xy[1]-a[1]-t*dy)
        progress=offset+t*length
        if distance<best[0]-1e-9 or (abs(distance-best[0])<=1e-9 and progress>best[1]):
            best=distance,progress
        offset+=length
    if not math.isfinite(best[0]):raise ValueError('Degenerate reference path')
    return best


class RouteGuard:
    def __init__(self, path, max_travel_m=16.):
        if not math.isfinite(max_travel_m) or not 0 < max_travel_m <= 16.:
            raise ValueError('Invalid travel budget')
        self.max_travel_m=max_travel_m
        self.path=path;self.peak_progress=0.;self.travel=0.;self.previous=None

    def update(self, xy):
        distance,progress=project(self.path,xy)
        if distance>.2:raise ValueError('Outside reviewed route corridor')
        if progress<self.peak_progress-.4:raise ValueError('Route progress regressed')
        if self.previous is not None:
            step=math.dist(xy,self.previous)
            if step>.5:raise ValueError('Robot pose jumped')
            self.travel+=step
        if self.travel>self.max_travel_m:raise ValueError('Travel budget exceeded')
        self.previous=list(xy);self.peak_progress=max(self.peak_progress,progress)
        return distance,progress


def can_control_valid(data, max_linear_raw=4):
    if max_linear_raw not in (3, 4):return False
    if len(data)!=8:return False
    vx,vy,wz=(int.from_bytes(data[i:i+1],'big',signed=True) for i in (1,2,3))
    return (data[0] in (0,1,2) and (data[0]==2 or vx==wz==0) and
            0<=vx<=max_linear_raw and vy==0 and abs(wz)<=2 and data[4]==0 and data[7]==0)
