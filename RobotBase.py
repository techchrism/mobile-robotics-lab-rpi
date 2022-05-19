import math


def rescale(val, in_min, in_max, out_min, out_max):
    return out_min + (val - in_min) * ((out_max - out_min) / (in_max - in_min))


class RobotBase:
    def __init__(self):
        self.position = (0.0, 0.0)
        self.angle = 0.0
        self.ultrasonic = (0.0, 0.0, 0.0, 0.0, 0.0)

    def set_position(self, position, angle):
        self.position = position
        self.angle = angle

    def set_ultrasonic(self, ultrasonic):
        self.ultrasonic = ultrasonic

    def avoid_obstacles(self):
        forward_multiplier = 0.5

        shortest_index = [i[0] for i in sorted(enumerate(self.ultrasonic), key=lambda x: x[1])][0]

        min_dist = 55
        if self.ultrasonic[shortest_index] < min_dist:
            turn_direction = 1 if shortest_index > 2 else -1
            # Bigger means closer
            closeness = min_dist - self.ultrasonic[shortest_index]
            turn_speed = rescale(closeness, 0, min_dist, 0.3, 2.5)
            forward_speed = rescale(max(self.ultrasonic[shortest_index] - 10, 0), 0, min_dist, 0, forward_multiplier)

            w = turn_direction * turn_speed
            v = forward_speed
            return v, w
        v = forward_multiplier
        w = 0.0
        return (v, w)

    def go_to_goal(self):
        goal = (-0.936, 1.708)

        distance_from_goal = math.sqrt(((self.position[0] - goal[0])**2) + ((self.position[1] - goal[1])**2))
        

        angle_from_bot = math.atan2((goal[0] - self.position[0]), (goal[1] - self.position[1])) * -1
        bot_angle = self.angle
        if(bot_angle > math.pi):
            bot_angle = bot_angle - (math.pi * 2)

        angle_distance = angle_from_bot - bot_angle

        print(f'{angle_distance}\t{distance_from_goal}\t{(goal[0] - self.position[0])}\t{(goal[1] - self.position[1])}')

        return 0, 0

    def get_velocity(self, telemetry_frame):
        if self.ultrasonic is None or self.position is None:
            return 0.0, 0.0

        if telemetry_frame is not None:
            telemetry_frame['pos'] = self.position
            telemetry_frame['angle'] = self.angle
            telemetry_frame['ultrasonic'] = self.ultrasonic

        coef = 0.477465

        v = 0.0
        w = 1.5 * math.pi

        # Hardcoded bc reasons
        goal = (0, 0)

        # Derive our displacement from where we want to go
        pos_error = (goal[0] - self.position[0], goal[1] - self.position[1])
        #print(f"Pos Err: {pos_error}")
        if telemetry_frame is not None:
            telemetry_frame['pos_error'] = pos_error

        # Derive distance to angle we *should* be at
        angle_error = math.atan2(pos_error[1], pos_error[0]) - self.angle
        #print(f"Ang Err: {angle_error * 180 / math.pi}")

        # Our angular velocity should scale down as we get closer
        # For this we choose w = a * t^2 for a nice quadratic
        # a is found to place our maximum velocity at maximum error
        w = coef * (angle_error ** 2)

        # The velocity should trail off as we get closer, but also
        #  if we are facing the wrong direction; we use two functions
        #  to control this: sqrt for getting closer, 1/x for turning
        #  around.  Both of these need to be cut off at a certain point
        closerCoef = min( (1.69031 * math.hypot(*pos_error)), 1 )
        pointingCoef = max( min( (1 / (angle_error + 0.693671)) - 0.441605, (1) ), 0)

        v = closerCoef * pointingCoef

        v, w = self.avoid_obstacles()
        #v, w = self.go_to_goal()
        #v, w = 0, 0
        v = v * -1

        return v, w
