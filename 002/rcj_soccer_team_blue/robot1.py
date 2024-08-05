# rcj_soccer_player controller - ROBOT B3

# Feel free to import built-in libraries
import math  # noqa: F401

# You can also import scripts that you put into the folder with controller
import utils
from rcj_soccer_robot import RCJSoccerRobot, TIME_STEP


#Set Robot Role
ROLE_FOLLOW = 0
ROLE_DEFENCE = 1
ROLE_ATTACK = 2
ROLE_MOVINGMIDDLE = 3
ROLE_STUCK = 4

BALLSTATE_NOBALL = 0
BALLSTATE_YESBALL = 1
BALLSTATE_BEHIND = 2
BALLSTATE_INFRONT = 3

TIME_STEP = 16

#This will be used to check if the robot is stuck
previous_positions = []

#ROBOT_DIRECTION_REVERSE = false #right to left is true / left to right is false

robot_state = ROLE_FOLLOW

print ("Running Robot 1 Code")

class Position():
    def __init__(self, ax = None,ay = None):
        self.x = ax
        self.y = ay
    x = None
    y = None

    def __str__(self):
        return "x" + str(self.x) + ", y" + str(self.y)

    def __copy__(self):
        return Position(self)

class MyRobot1(RCJSoccerRobot):

    #def __init__(self):
    #    #check which side i am on (Yellow or Blue) Yellow is false
    #    self. ROBOT_DIRECTION_REVERSE


    # Function to normalize angles
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Function to get robot's heading
    def get_heading(self):
        compass_values = self.compass.getValues()
        #compass_values = self.get_compass_heading()
        return math.atan2(compass_values[0], compass_values[2])

    # Function to get current position
    def get_position(self):
        gps_values = gps.getValues()
        return gps_values[0], gps_values[2]  # assuming Y is up

    # Function to transform the direction vector to the global coordinate system
    def transform_direction_vector(self, direction, heading):
        dx_local, dy_local, dz_local = direction
        sin_heading = math.sin(heading)
        cos_heading = math.cos(heading)

        # Transform to global coordinates
        dx_global = cos_heading * dx_local - sin_heading * dz_local
        dy_global = dy_local  # Typically, this would be the height component
        dz_global = sin_heading * dx_local + cos_heading * dz_local

        return dx_global, dy_global, dz_global

    # Function to calculate the ball's position
    def calculate_ball_position(self, robot_position, direction, strength):
        # Typically, the strength can be used to estimate the distance
        # Let's assume strength is proportional to the inverse of distance
        estimated_distance = 1 / (strength + 1e-6)  # Add a small value to avoid division by zero

        heading = self.get_heading()
        dx_global, _, dz_global = self.transform_direction_vector(direction, heading)

        # Current position of the robot
        robot_x, robot_y = robot_position

        # Estimate ball position in global coordinates
        ball_x = robot_x + dx_global * estimated_distance
        ball_y = robot_y + dz_global * estimated_distance

        return ball_x, ball_y

    def get_distance_between_two_positions(self, pos1_x, pos1_y, pos2_x, pos2_y):
        return math.sqrt((pos1_x - pos2_x)**2 + (pos1_y - pos2_y)**2)

    # Function to turn the robot towards the ball
    def turn_to_ball(self, pos1_x, pos1_y, pos2_x, pos2_y):
        # Get current robot's position
        robot_x, robot_y = pos1_x, pos1_y

        print ("i can see the ball and turning towards it")

        # Get ball's position
        ball_x, ball_y = pos2_x, pos2_y

        # Calculate angle to the ball
        delta_x = ball_x - robot_x
        delta_y = ball_y - robot_y
        angle_to_ball = math.atan2(delta_y, delta_x)

        # Get robot's current heading
        robot_heading = self.get_compass_heading()

        # Calculate the angle difference
        angle_difference = self.normalize_angle(angle_to_ball - robot_heading)

        # Rotate the robot towards the ball
        if abs(angle_difference) > 0.1:  # Small threshold to stop turning
            if angle_difference > 0:
                self.set_speed(-1.0, 1.0)  # Turn left
            else:
                self.set_speed(1.0, -1.0)  # Turn right
        else:
            self.set_speed(0.0, 0.0)  # Stop turning

    # Function to set wheel speeds
    def set_speed(self, left_speed, right_speed):
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
        print ("setting speed")

    # Function to normalize angles
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    # Function to perform a scanning rotation
    def scan_for_ball(self):
        # Rotate the robot in place to look for the ball
        self.set_speed(-6.0, 6.0)  # Rotate left
        start_time = self.robot.getTime()
        while self.robot.step(TIME_STEP) != -1 and (self.robot.getTime() - start_time) < 2:  # Rotate for 3 seconds
            print ("turning around")
            if self.is_new_ball_data():
                return True
        self.set_speed(0.0, 0.0)
        return False

    # Function to move to a target position
    def move_to_target(self, target_x, target_y):
        current_x, current_y = self.get_gps_coordinates()

        print ("i am trying to move to the target")

        delta_x = target_x - current_x
        delta_y = target_y - current_y
        angle_to_target = math.atan2(delta_y, delta_x)

        print ("angle_to_target", angle_to_target)

        current_heading = self.get_compass_heading()
        print ("current heading ", current_heading)

        angle_difference = self.normalize_angle(angle_to_target - current_heading)
        print ("angle difference: ", angle_difference)

        angle_threshold = 0.1
        base_turn_speed = 4
        proportional_gain = 0.5

        # Adjust turning speed based on angle difference
        turn_speed = base_turn_speed + proportional_gain * abs(angle_difference)

        if abs(angle_difference) > angle_threshold:  # Turn towards target
            #if angle_difference > 0 and angle_difference < 3.1:
                #self.set_speed(-1.0, 1.0)  # Turn left
                self.set_speed(-turn_speed, turn_speed)  # Turn left
            #else:
            #    self.set_speed(turn_speed, -turn_speed)  # Turn right
        else:
            self.set_speed(5.0, 5.0)  # Move forward

        # Stop if close to the target
        if self.get_distance_between_two_positions(current_x, current_y, target_x, target_y) < 0.2:
            self.set_speed(0.0, 0.0)
            #self.robot_state = ROLE_FOLLOW

    def set_position(self, x, y):
        self.x, self.y = x, y

    # Function to move to a target position
    def move_until_at_target(self, target_x, target_y):

        #self.left_motor.setVelocity(10.0)
        #self.right_motor.setVelocity(10.0)

        current_x, current_y = self.get_gps_coordinates()
        current_heading = self.get_compass_heading()
        print ("current heading ", current_heading)

        print(f"Current position: ({current_x}, {current_y})")
        print(f"Target position: ({target_x}, {target_y})")
        print ("i am NOW trying to move to the target")

        # Calculate direction vector
        dx = target_x - current_x
        dy = target_y - current_y
        distance_to_target = math.sqrt(dx**2 + dy**2)

        print("Distance to the target: ", distance_to_target)

        if distance_to_target > 0.2:
            # Normalize direction vector
            if distance_to_target != 0:
                direction_x = dx / distance_to_target
                direction_y = dy / distance_to_target
            else:
                direction_x = 0
                direction_y = 0

            print(f"Direction vector: ({direction_x}, {direction_y})")

            # Calculate angle to the target
            angle_to_target = math.atan2(dy, dx)
            angle_difference = self.normalize_angle(angle_to_target - current_heading)
            distance_to_target = math.sqrt(dx ** 2 + dy ** 2)

            print(f"Angle to target: {math.degrees(angle_to_target)} degrees")
            print(f"Angle difference: {math.degrees(angle_difference)} degrees")

            proportional_gain = 0.75
            forward_speed = 5.0  # Forward movement speed
            turning_speed = angle_difference * proportional_gain  # Adjust proportional gain as needed

            # Calculate individual wheel speeds
            left_speed = forward_speed - turning_speed
            right_speed = forward_speed + turning_speed
        else:
            left_speed = 1
            right_speed = -1

        return left_speed, right_speed


    def normalize_vector_2d(self, vector):
        """
        Normalize a 2D vector to a unit vector.

        Args:
        vector (tuple): The 2D vector (vx, vy).

        Returns:
        tuple: The normalized 2D vector.
        """
        vx, vy = vector
        magnitude = math.sqrt(vx ** 2 + vy ** 2)
        if magnitude == 0:
            return (0, 0)  # Avoid division by zero if magnitude is 0
        return (vx / magnitude, vy / magnitude)

    def calculate_normalized_direction(self, pos1_x, pos1_y, pos2_x, pos2_y):
        """
        Calculate the normalized direction vector from the robot to a point in 2D.

        Args:
        robot_pos (tuple): The robot's position (x1, y1).
        point_pos (tuple): The target point's position (x2, y2).

        Returns:
        tuple: The normalized direction vector (ux, uy).
        """
        x1, y1 = pos1_x, pos1_y
        x2, y2 = pos2_x, pos2_y

        # Calculate the difference vector
        dx = x2 - x1
        dy = y2 - y1

        # Normalize the vector
        return self.normalize_vector_2d((dx, dy))

    def run(self):

        if self.team.upper() == 'B':
            r_direction = 1
        else:
            r_direction = -1


        robot_state = ROLE_FOLLOW
        stuc_threshold = 10

        while self.robot.step(TIME_STEP) != -1:
            if self.is_new_data():
                print (robot_state)

                data = self.get_new_data()  # noqa: F841

                while self.is_new_team_data():
                    team_data = self.get_new_team_data()  # noqa: F841
                    # Do something with team data

                if self.is_new_ball_data():
                    ball_data = self.get_new_ball_data()
                    ball_state = BALLSTATE_YESBALL
                    print ("yes ball")
                else:
                    print ("no ball")
                    # If the robot does not see the ball, stop motors
                    self.left_motor.setVelocity(0)
                    self.right_motor.setVelocity(0)
                    ball_state = BALLSTATE_NOBALL
                    #continue

                # Get data from compass
                heading = self.get_compass_heading()  # noqa: F841
                print("heading: ", heading)
                # Get GPS coordinates of the robot
                robot_pos = self.get_gps_coordinates()  # noqa: F841

                # Get Robot position x and y
                robot_x = robot_pos[0]
                robot_y = robot_pos[1]

                previous_positions.append((round(robot_x, 3), round(robot_y,3)))
                if len(previous_positions) > stuc_threshold:
                        previous_positions.pop(0)

                if len(set(previous_positions))  == 1:
                    print ("robot has not moved")
                    left_speed = -10
                    right_speed = -10

                    prev_state = robot_state
                    robot_state = ROLE_STUCK
                else:

                    print (previous_positions)

                    # Get data from sonars
                    sonar_values = self.get_sonar_values()  # noqa: F841

                    if ball_state == BALLSTATE_YESBALL:

                        print( "I can see the ball")

                        # Compute the speed for motors
                        direction = utils.get_direction(ball_data["direction"])

                        # get me the ball position please
                        ball_direction = ball_data['direction']
                        ball_strength = ball_data['strength']

                        ball_x, ball_y = self.calculate_ball_position(robot_pos, ball_direction, ball_strength)

                        print ("ball_x ", ball_x)
                        print ("ball_y ", ball_y)

                        ball_distance = self.get_distance_between_two_positions( robot_x, robot_y, ball_x, ball_y )
                        print( "distance to the ball ", ball_distance)

                        self.turn_to_ball(robot_x, robot_y, ball_x, ball_y )

                        # If the robot has the ball right in front of it, go forward,
                        # rotate otherwise
                        if direction == 0:
                            #check if we are heading towards our own goal - check both ends of the field
                            #if self.get_compass_heading() > 3:
                            #    left_speed = direction * 0.5
                            #    right_speed = direction * -0.5
                            #    print ("here")
                            #else:
                            left_speed = 10
                            right_speed = 10
                        else:
                            left_speed = direction * 4
                            right_speed = direction * -4

                        # Set the speed to motors
                        self.left_motor.setVelocity(left_speed)
                        self.right_motor.setVelocity(right_speed)

                        # Send message to team robots
                        self.send_data_to_team(self.player_id)

                        robot_state = ROLE_FOLLOW

                    if ball_state == BALLSTATE_NOBALL:
                        print("i can not see the ball")

                        ##left_speed, right_speed = self.move_until_at_target(0,0.5)

                        y_back_pos_target = 0.3 * r_direction

                        if robot_state == ROLE_MOVINGMIDDLE:
                            print ("trying to move to the position i was given")
                            ##left_speed, right_speed = self.move_until_at_target(0, 0.3)
                            left_speed, right_speed = self.move_until_at_target(0, y_back_pos_target)
                            ##self.move_to_target(0.0, 0.0)
                            #if self.get_distance_between_two_positions(robot_x, robot_y, 0.0, 0.0) < 0.2:
                            #    self.set_speed(0.0, 0.0)
                        else:
                            print("i can not see the ball")
                            found_ball = self.scan_for_ball()
                            if not found_ball:
                                # Move to a strategic location (e.g., midfield)
                                print ("move to position")
                                robot_state = ROLE_MOVINGMIDDLE
                                ###left_speed, right_speed = self.move_until_at_target(0, 0.3)
                                left_speed, right_speed = self.move_until_at_target(0, y_back_pos_target)
                                ##self.move_to_target(0.0, 0.0)

                try:
                    left_speed
                except NameError:
                    left_speed = 1

                try:
                    right_speed
                except NameError:
                    right_speed = 1

                if left_speed > 10:
                    left_speed = 10
                if right_speed > 10:
                    right_speed = 0

                self.left_motor.setVelocity(left_speed)
                self.right_motor.setVelocity(right_speed)