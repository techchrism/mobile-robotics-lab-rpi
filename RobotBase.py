import math

STATE_GO_TO_GOAL = 'go to goal'
STATE_AVOID_OBSTACLES = 'avoid obstacles'
STATE_BLEND_GOAL_OBSTACLES = 'blend goal and obstacles'
STATE_DONE = 'done'


def rescale(val, in_min, in_max, out_min, out_max):
    return out_min + (val - in_min) * ((out_max - out_min) / (in_max - in_min))


class RobotBase:
    position = (0.0, 0.0)
    angle = 0.0
    ultrasonic = (0.0, 0.0, 0.0, 0.0, 0.0)
    weighted_ultrasonic = (0.0, 0.0, 0.0, 0.0, 0.0)
    goal = (0, 1)
    state = STATE_GO_TO_GOAL

    def set_position(self, position, angle):
        self.position = position
        self.angle = angle

    def set_ultrasonic(self, ultrasonic):
        self.ultrasonic = ultrasonic
        weights = [3, 1, 1, 1, 3]
        self.weighted_ultrasonic = [weights[i] * ultrasonic[i] for i in range(len(ultrasonic))]

    def avoid_obstacles(self):
        forward_multiplier = 0.5

        shortest_index = [i[0] for i in sorted(enumerate(self.weighted_ultrasonic), key=lambda x: x[1])][0]

        min_dist = 55
        if self.weighted_ultrasonic[shortest_index] < min_dist:
            turn_direction = 1 if shortest_index > 2 else -1
            # Bigger means closer
            closeness = min_dist - self.weighted_ultrasonic[shortest_index]
            turn_speed = rescale(closeness, 0, min_dist, 0.3, 2.5)
            forward_speed = rescale(max(self.weighted_ultrasonic[shortest_index] - 10, 0), 0, min_dist, 0, forward_multiplier)

            w = turn_direction * turn_speed
            v = forward_speed
            return v, w
        v = forward_multiplier
        w = 0.0
        return v, w

    def get_distance_from_goal(self):
        return math.sqrt(((self.position[0] - self.goal[0])**2) + ((self.position[1] - self.goal[1])**2))

    def go_to_goal(self):
        distance_from_goal = self.get_distance_from_goal()

        angle_from_bot = math.atan2((self.goal[0] - self.position[0]), (self.goal[1] - self.position[1]))
        bot_angle = self.angle
        if bot_angle > math.pi:
            bot_angle = bot_angle - (math.pi * 2)

        angle_distance = math.atan2(math.sin(angle_from_bot - bot_angle), math.cos(angle_from_bot - bot_angle))

        max_turn_speed = 1
        min_turn_speed = 0.5
        power_from_distance = rescale(min(distance_from_goal, 1), 0, 1, 0, 0.5)
        turn_speed = rescale(abs(angle_distance), 0, math.pi, min_turn_speed, max_turn_speed)
        power_from_turn = max(rescale(abs(angle_distance), 0, (math.pi / 5), power_from_distance, power_from_distance / 5), power_from_distance / 5)

        w = turn_speed
        if angle_distance > 0:
            w *= -1
        v = power_from_turn

        return v, w

    def blend(self, a, b, a_weight):
        v_a, w_a = a
        v_b, w_b = b
        v = (v_a * a_weight) + (v_b * (1 - a_weight))
        w = (w_a * a_weight) + (w_b * (1 - a_weight))
        return v, w

    def get_velocity(self, telemetry_frame):
        if self.ultrasonic is None or self.position is None:
            return 0.0, 0.0

        distance_from_goal = self.get_distance_from_goal()

        if telemetry_frame is not None:
            telemetry_frame['pos'] = self.position
            telemetry_frame['angle'] = self.angle
            telemetry_frame['ultrasonic'] = self.ultrasonic
            telemetry_frame['weighted_ultrasonic'] = self.weighted_ultrasonic
            telemetry_frame['state'] = self.state
            telemetry_frame['goal'] = self.goal
            telemetry_frame['goal_distance'] = distance_from_goal

        blend_goal_obstacles_threshold = 50
        avoid_obstacles_threshold = 20
        done_threshold = 0.05

        if self.state == STATE_DONE:
            return 0, 0

        # Avoid obstacles when too close
        if min(self.weighted_ultrasonic) < avoid_obstacles_threshold and self.state != STATE_AVOID_OBSTACLES:
            self.state = STATE_AVOID_OBSTACLES
            return self.get_velocity(telemetry_frame)

        # Switch to done state if within range of goal
        if distance_from_goal < 0.1 and self.state != STATE_DONE:
            self.state = STATE_DONE
            return self.get_velocity(telemetry_frame)

        if self.state == STATE_GO_TO_GOAL:
            # Switch to avoid obstacles if too close
            if min(self.weighted_ultrasonic) < blend_goal_obstacles_threshold:
                self.state = STATE_BLEND_GOAL_OBSTACLES
                return self.get_velocity(telemetry_frame)
            else:
                v, w = self.go_to_goal()

        if self.state == STATE_BLEND_GOAL_OBSTACLES:
            if min(self.weighted_ultrasonic) > blend_goal_obstacles_threshold:
                self.state = STATE_GO_TO_GOAL
                return self.get_velocity(telemetry_frame)
            v, w = self.blend(self.avoid_obstacles(), self.go_to_goal(), 0.5)

        if self.state == STATE_AVOID_OBSTACLES:
            if min(self.weighted_ultrasonic) > avoid_obstacles_threshold:
                self.state = STATE_GO_TO_GOAL
                return self.get_velocity(telemetry_frame)
            else:
                v, w = self.avoid_obstacles()

        v = v * -1

        return v, w
