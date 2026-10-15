# Formation geometry and coordinated motion in a common simulation NED frame
import math
import heapq
import random
import time

import rclpy

from .obstacle_tests import launch_configuration
from .spinning import spin_for


def validate_points(points, count, separation=.8):
    if len(points) != count or any(len(p) != 3 or not all(math.isfinite(x) for x in p) for p in points):
        raise ValueError('Provide one finite NED target per drone.')
    if not math.isfinite(separation) or separation <= 0:
        raise ValueError('Minimum separation must be positive and finite.')
    if any(math.dist(a, b) < separation for i, a in enumerate(points) for b in points[i+1:]):
        raise ValueError('Targets violate minimum drone separation.')


def formation_targets(shape, count, center, spacing=1.5):
    if not math.isfinite(spacing) or spacing < .8:
        raise ValueError('Formation spacing must be finite and at least 0.8 m.')
    n, e, d = center
    if shape == 'line':
        return [(n, e + (i-(count-1)/2)*spacing, d) for i in range(count)]
    if shape == 'grid':
        cols = math.ceil(math.sqrt(count))
        offsets = [(i//cols*spacing, i%cols*spacing) for i in range(count)]
        mn = sum(p[0] for p in offsets)/count
        me = sum(p[1] for p in offsets)/count
        return [(n+x-mn, e+y-me, d) for x,y in offsets]
    if shape == 'circle':
        if count == 1:
            return [center]
        radius = spacing / (2*math.sin(math.pi/count))
        return [(n+radius*math.cos(2*math.pi*i/count),
                 e+radius*math.sin(2*math.pi*i/count), d) for i in range(count)]
    raise ValueError('Formation must be line, grid, or circle.')


def optimal_slots(starts, slots):
    # Assign horizontal slots with minimum total squared travel (Hungarian)
    count = len(starts)
    costs = [[sum((p[k]-q[k])**2 for k in (0, 1)) for q in slots] for p in starts]
    u, v = [0.]*(count+1), [0.]*(count+1)
    matched, previous = [0]*(count+1), [0]*(count+1)
    for row in range(1, count+1):
        matched[0] = row
        column = 0
        minimum, used = [math.inf]*(count+1), [False]*(count+1)
        while True:
            used[column] = True
            active = matched[column]
            delta, next_column = math.inf, 0
            for candidate in range(1, count+1):
                if not used[candidate]:
                    cost = costs[active-1][candidate-1]-u[active]-v[candidate]
                    if cost < minimum[candidate]:
                        minimum[candidate], previous[candidate] = cost, column
                    if minimum[candidate] < delta:
                        delta, next_column = minimum[candidate], candidate
            for candidate in range(count+1):
                if used[candidate]:
                    u[matched[candidate]] += delta
                    v[candidate] -= delta
                else:
                    minimum[candidate] -= delta
            column = next_column
            if matched[column] == 0:
                break
        while column:
            predecessor = previous[column]
            matched[column] = matched[predecessor]
            column = predecessor
    assigned = [None]*count
    for column in range(1, count+1):
        row = matched[column]-1
        slot = slots[column-1]
        assigned[row] = tuple(slot)
    return assigned


def planar_path(start, goal, obstacles, separation=1.05):
    # Shortest visible route around stationary drones, at a fixed altitude
    def clear(a, b):
        dn, de = b[0]-a[0], b[1]-a[1]
        length2 = dn*dn+de*de
        for p in obstacles:
            t = max(0., min(1., ((p[0]-a[0])*dn+(p[1]-a[1])*de)/length2)) if length2 else 0.
            if math.hypot(a[0]+t*dn-p[0], a[1]+t*de-p[1]) < separation:
                return False
        return True
    if clear(start, goal):
        return [goal]
    nodes = [start, goal]
    radius = separation/math.cos(math.pi/16)+.05
    for p in obstacles:
        for i in range(16):
            angle = 2*math.pi*i/16
            candidate = (p[0]+radius*math.cos(angle), p[1]+radius*math.sin(angle), goal[2])
            if clear(candidate, candidate):
                nodes.append(candidate)
    distances, previous = {0: 0.}, {}
    queue = [(0., 0)]
    while queue:
        distance, i = heapq.heappop(queue)
        if distance > distances[i]:
            continue
        if i == 1:
            path = []
            while i:
                path.append(nodes[i])
                i = previous[i]
            return path[::-1]
        for j, point in enumerate(nodes):
            candidate_distance = distance+math.dist(nodes[i], point)
            if candidate_distance < distances.get(j, math.inf) and clear(nodes[i], point):
                distances[j], previous[j] = candidate_distance, i
                heapq.heappush(queue, (candidate_distance, j))
    raise ValueError('Cannot find a separated horizontal route between drones.')


class SwarmBehaviors:
    def __init__(self, controller):
        self.controller = controller
        self.origins = None
        self.flight_down = None

    def configure(self):
        configurations = [launch_configuration(d) for d in self.controller.drones]
        if len({c[0] for c in configurations}) != 1 or not configurations[0][0]:
            raise RuntimeError('Use the shared simulation launcher for common-frame behaviors.')
        self.spawn_positions = [(y, 0., 0.) for _, y in configurations]
        # The shared launcher uses x500_vision with world-frame external vision.
        # PX4 already reports the spawn displacement in local NED. Adding it
        # again corrupts measured separation and can send members to one point.
        self.origins = [(0., 0., 0.) for _ in configurations]

    def positions(self, timeout_s=10):
        if self.origins is None:
            self.configure()
        requested = time.monotonic()
        deadline = requested + timeout_s
        while time.monotonic() < deadline:
            spin_for(self.controller, .02)
            if all(d.state is not None and d.state_received_at > requested
                   and time.monotonic()-d.state_received_at < 1
                   and d.armed is True and d.offboard is True for d in self.controller.drones):
                return [tuple(v+o for v,o in zip((d.state.pose.pose.position.x,
                    d.state.pose.pose.position.y, d.state.pose.pose.position.z), origin))
                    for d,origin in zip(self.controller.drones, self.origins)]
        raise TimeoutError('Fresh armed OFFBOARD telemetry is required from every drone.')

    def centroid(self):
        positions = self.positions()
        return tuple(sum(p[k] for p in positions)/len(positions) for k in range(3))

    def move_to(self, targets, *, sample=None, phase='movement', timeout_s=90, separation=.8,
                mode='position', speed_mps=1.0):
        if mode not in ('position', 'velocity'):
            raise ValueError('mode must be position or velocity.')
        if not math.isfinite(speed_mps) or speed_mps <= 0:
            raise ValueError('speed_mps must be finite and positive.')
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('timeout_s must be finite and positive.')
        targets = [tuple(p) for p in targets]
        validate_points(targets, self.controller.drone_count, separation)
        starts = self.positions()
        validate_points(starts, len(starts), separation)
        # Reject intersecting straight trajectories; staged moves route horizontally
        for i in range(len(starts)):
            for j in range(i):
                a = tuple(starts[i][k]-starts[j][k] for k in range(3))
                b = tuple(targets[i][k]-starts[i][k]-targets[j][k]+starts[j][k] for k in range(3))
                bb = sum(v*v for v in b)
                t = max(0., min(1., -sum(x*y for x,y in zip(a,b))/bb)) if bb else 0.
                if math.sqrt(sum((x+t*y)**2 for x,y in zip(a,b))) < separation:
                    raise ValueError('Paths cross; use staged=True for formation changes.')
        sent = time.monotonic()
        # Use a per-member progress watchdog: Gazebo can run much slower than
        # wall time. Another member's movement must not hide a stuck drone.
        progress_errors = [math.dist(p, t) for p, t in zip(starts, targets)]
        progress_at = [sent] * len(starts)
        settled = None
        held = False
        try:
            while True:
                spin_for(self.controller)
                if any(time.monotonic()-d.state_received_at >= 1
                       for d in self.controller.drones):
                    # A busy DDS graph can briefly delay delivery. Hold every
                    # member before servicing the queue; never keep travelling
                    # while feedback is stale.
                    self.controller.command_all('stop')
                    held = True
                    try:
                        self.positions(timeout_s=2)
                    except TimeoutError as exc:
                        raise TimeoutError('Swarm telemetry remained stale after holding.') from exc
                complete = True
                points = []
                current_points = [tuple(x+o for x,o in zip(
                    (d.state.pose.pose.position.x, d.state.pose.pose.position.y,
                     d.state.pose.pose.position.z), origin))
                    for d, origin in zip(self.controller.drones, self.origins)]
                largest_error = max(math.dist(point, goal)
                                    for point, goal in zip(current_points, targets))
                scale = min(1., speed_mps / max(largest_error, 1e-9))
                for drone, origin, target in zip(self.controller.drones, self.origins, targets):
                    if drone.armed is not True or drone.offboard is not True:
                        raise RuntimeError('A drone lost armed OFFBOARD mode.')
                    if time.monotonic()-drone.state_received_at >= 1:
                        raise TimeoutError(f'Swarm telemetry became stale for drone '
                                           f'{self.controller.drones.index(drone)}.')
                    p = drone.state.pose.pose.position
                    v = drone.state.twist.twist.linear
                    point = tuple(x+o for x,o in zip((p.x,p.y,p.z), origin))
                    points.append(point)
                    if mode == 'position':
                        held = False
                        drone.move_to(*(x-o for x,o in zip(target, origin)))
                    else:
                        held = False
                        error = [x-p for x,p in zip(target, point)]
                        # Brake near the destination, with a common fleet speed cap.
                        drone.set_velocity(*(scale*x for x in error))
                    if (drone.state_received_at <= sent or math.dist(point,target) > .1
                            or math.hypot(v.x,v.y,v.z) > .15):
                        complete = False
                if sample:
                    sample(phase, targets, points)
                validate_points(points, len(points), separation)
                if complete:
                    settled = settled or time.monotonic()
                    if time.monotonic()-settled >= .5:
                        if mode == 'velocity':
                            self.controller.command_all('stop')
                        return
                else:
                    settled = None
                now = time.monotonic()
                stalled = []
                for i, (point, target) in enumerate(zip(points, targets)):
                    error = math.dist(point, target)
                    v = self.controller.drones[i].state.twist.twist.linear
                    speed = math.hypot(v.x, v.y, v.z)
                    if error <= .1 and speed <= .15:
                        progress_at[i] = now
                    elif error <= progress_errors[i] - .05:
                        # Cumulative five-centimetre improvement avoids letting
                        # hover noise or repeated back-and-forth reset the timer.
                        progress_errors[i] = error
                        progress_at[i] = now
                    if now - progress_at[i] >= timeout_s:
                        stalled.append(f'drone {i}: target NED={target}, actual NED={point}, '
                                       f'error={error:.3f} m, speed={speed:.3f} m/s')
                if stalled:
                    raise TimeoutError(f'Swarm destinations did not settle in phase {phase!r}; '
                                       f'no arrival or 0.05 m progress for {timeout_s:.1f}s: '
                                       + '; '.join(stalled))
        except BaseException:
            try:
                if not held:
                    self.controller.command_all('stop')
            except Exception:
                pass
            raise

    def go_to(self, targets, *, staged=True, sample=None, phase='formation',
              mode='position', speed_mps=1.0):
        targets = [tuple(p) for p in targets]
        validate_points(targets, self.controller.drone_count)
        if not staged:
            return self.move_to(targets, sample=sample, phase=phase, mode=mode, speed_mps=speed_mps)
        starts = self.positions()
        height = self.flight_down if self.flight_down is not None else sum(p[2] for p in starts)/len(starts)
        targets = [(p[0], p[1], height) for p in targets]
        validate_points(targets, len(starts))
        points = [(p[0], p[1], height) for p in starts]
        validate_points(points, len(points))
        clearance = min([1.05, *[math.dist(a,b) for collection in (points,targets)
                                for i,a in enumerate(collection) for b in collection[i+1:]]])
        def direct_paths_clear():
            for i, a in enumerate(points):
                for j, b in enumerate(points[:i]):
                    delta = [a[k]-b[k] for k in (0, 1)]
                    velocity = [targets[i][k]-a[k]-targets[j][k]+b[k] for k in (0, 1)]
                    speed2 = sum(v*v for v in velocity)
                    t = max(0., min(1., -sum(x*v for x,v in zip(delta,velocity))/speed2)) if speed2 else 0.
                    if math.hypot(*(x+t*v for x,v in zip(delta,velocity))) < 1.05:
                        return False
            return True
        if direct_paths_clear():
            self.flight_down = height
            return self.move_to(targets, sample=sample, phase=phase, mode=mode, speed_mps=speed_mps)
        # Vacate occupied destination slots first. Parking and all detours stay
        # on the flight plane; only one member moves while its neighbors hold
        north = min(p[0] for p in points+targets)-3.
        east = min(p[1] for p in points+targets)
        parking = [(north, east+2*i, height) for i in range(len(points))]
        planned = []
        for destinations in (parking, targets):
            for i, destination in enumerate(destinations):
                obstacles = [p for j,p in enumerate(points) if j != i]
                for point in planar_path(points[i], destination, obstacles, separation=clearance):
                    points[i] = point
                    planned.append(list(points))
        self.flight_down = height
        for points in planned:
            self.move_to(points, sample=sample, phase=phase+'_horizontal', mode=mode, speed_mps=speed_mps)

    def formation(self, shape='line', spacing_m=1.5, center=None, **kwargs):
        starts = self.positions()
        if center is None and self.flight_down is not None:
            center = (sum(p[0] for p in starts)/len(starts),
                      sum(p[1] for p in starts)/len(starts), self.flight_down)
        center = (tuple(sum(p[k] for p in starts)/len(starts) for k in range(3))
                  if center is None else tuple(center))
        self.flight_down = center[2]
        slots = formation_targets(shape, self.controller.drone_count, center, spacing_m)
        lineup = getattr(self, 'spawn_positions', self.origins)
        if shape == 'line' and lineup and len(lineup) > 1:
            # A line is undirected: the opposite lineup orientation is perpendicular
            dn = lineup[-1][0]-lineup[0][0]
            de = lineup[-1][1]-lineup[0][1]
            length = math.hypot(dn, de)
            if length:
                slots = [(center[0]-de/length*(i-(len(starts)-1)/2)*spacing_m,
                          center[1]+dn/length*(i-(len(starts)-1)/2)*spacing_m,
                          center[2]) for i in range(len(starts))]
        targets = optimal_slots(starts, slots)
        # All members share the formation's commanded altitude
        self.go_to(targets, **kwargs)
        return targets

    def translate(self, north=0., east=0., down=0., **kwargs):
        if not all(math.isfinite(v) for v in (north,east,down)):
            raise ValueError('Translation must be finite.')
        points = self.positions()
        height = self.flight_down if self.flight_down is not None else sum(p[2] for p in points)/len(points)
        targets = [(p[0]+north, p[1]+east, height+down) for p in points]
        self.flight_down = height+down
        self.move_to(targets, **kwargs)
        return targets

    def randomize(self, radius_m=3., seed=None, **kwargs):
        if not math.isfinite(radius_m) or radius_m <= 0:
            raise ValueError('Randomization radius must be positive and finite.')
        rng = random.Random(seed)
        starts = self.positions()
        height = self.flight_down if self.flight_down is not None else sum(p[2] for p in starts)/len(starts)
        targets = []
        for point in starts:
            for _ in range(1000):
                candidate = (point[0]+rng.uniform(-radius_m,radius_m),
                             point[1]+rng.uniform(-radius_m,radius_m),height)
                def clear_path(other_start, other_target):
                    a = tuple(point[k]-other_start[k] for k in range(3))
                    b = tuple(candidate[k]-point[k]-other_target[k]+other_start[k] for k in range(3))
                    bb = sum(v*v for v in b)
                    t = max(0., min(1., -sum(x*y for x,y in zip(a,b))/bb)) if bb else 0.
                    return math.sqrt(sum((x+t*y)**2 for x,y in zip(a,b))) >= .8
                if (all(math.dist(candidate,p) >= 1.5 for p in targets)
                        and all(clear_path(s,t) for s,t in zip(starts,targets))):
                    targets.append(candidate)
                    break
            else:
                raise ValueError('Cannot fit separated random targets; increase radius.')
        self.go_to(targets, **kwargs)
        return targets

    def split(self, direction='left', spacing_m=1.5, **kwargs):
        if direction not in ('left','right'):
            raise ValueError('Direction must be left or right.')
        points = self.positions()
        height = self.flight_down if self.flight_down is not None else sum(p[2] for p in points)/len(points)
        count = len(points)
        if count < 3:
            raise ValueError('Majority/minority split requires at least three drones.')
        if not math.isfinite(spacing_m) or spacing_m < .8:
            raise ValueError('Split spacing must be at least 0.8 m.')
        majority = count//2+1
        sign = -1 if direction == 'left' else 1
        targets = [(p[0]+sign*(1 if i<majority else -1)*max(spacing_m,abs(i-(count-1)/2)*spacing_m),
                    p[1],height) for i,p in enumerate(points)]
        self.go_to(targets, **kwargs)
        return targets
