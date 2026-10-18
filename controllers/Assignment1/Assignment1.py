
from controller import Supervisor
import math

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# --------------------------------
# 1. MOTOR SETUP
# --------------------------------

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# --------------------------------
# 2. WHEEL ENCODERS
# --------------------------------

left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

# --------------------------------
# 3. GROUND TRUTH
# --------------------------------

# Used only to evaluate localization accuracy
robot_node = robot.getSelf()
translation_field = robot_node.getField("translation")

# --------------------------------
# 4. ROBOT PARAMETERS
# --------------------------------

WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.052
SPEED = 3.0

# --------------------------------
# 5. INITIAL ESTIMATED POSE
# --------------------------------

x = 0.0
y = 0.0
theta = 0.0

# Initialize sensors
robot.step(timestep)

previous_left = left_encoder.getValue()
previous_right = right_encoder.getValue()

# Record initial ground-truth pose
initial_position = translation_field.getSFVec3f()
initial_orientation = robot_node.getOrientation()

print("Initial position:", initial_position)
print("Initial orientation:", initial_orientation)

# --------------------------------
# 6. ODOMETRY LOCALIZATION
# --------------------------------

def update_odometry():
    global x, y, theta
    global previous_left, previous_right

    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    # Change in wheel rotation
    delta_left = current_left - previous_left
    delta_right = current_right - previous_right

    # Wheel travel distances
    distance_left = WHEEL_RADIUS * delta_left
    distance_right = WHEEL_RADIUS * delta_right

    # Robot movement
    delta_distance = (distance_left + distance_right) / 2
    delta_theta = (distance_right - distance_left) / WHEEL_DISTANCE

    # Update estimated position and heading
    x += delta_distance * math.cos(theta + delta_theta / 2)
    y += delta_distance * math.sin(theta + delta_theta / 2)
    theta += delta_theta

    # Save encoder readings
    previous_left = current_left
    previous_right = current_right

# --------------------------------
# 7. ROBOT MOVEMENT
# --------------------------------

def move(left_speed, right_speed, duration):
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    steps = int(duration * 1000 / timestep)

    for _ in range(steps):
        if robot.step(timestep) == -1:
            return

        update_odometry()

    print(
        f"Estimated pose: x={x:.3f} m, "
        f"y={y:.3f} m, "
        f"theta={math.degrees(theta):.1f} deg"
    )

# --------------------------------
# 8. COORDINATE SYSTEM CHECK
# --------------------------------

def check_coordinate_system():
    actual_position = translation_field.getSFVec3f()

    dx = actual_position[0] - initial_position[0]
    dy = actual_position[1] - initial_position[1]

    print("\n--- Coordinate System Check ---")
    print(f"Actual X displacement: {dx:.4f} m")
    print(f"Actual Y displacement: {dy:.4f} m")

    print(
        "Initial local +X direction:",
        initial_orientation[0],
        initial_orientation[3]
    )

    print(
        "Initial local +Y direction:",
        initial_orientation[1],
        initial_orientation[4]
    )

# --------------------------------
# 9. DISPLACEMENT EVALUATION
# --------------------------------

def evaluate_localization():
    actual_position = translation_field.getSFVec3f()

    actual_dx = actual_position[0] - initial_position[0]
    actual_dy = actual_position[1] - initial_position[1]

    actual_distance = math.sqrt(actual_dx**2 + actual_dy**2)
    estimated_distance = math.sqrt(x**2 + y**2)

    distance_error = abs(estimated_distance - actual_distance)

    print("\n--- Localization Evaluation ---")
    print(f"Estimated distance: {estimated_distance:.3f} m")
    print(f"Actual distance: {actual_distance:.3f} m")
    print(f"Distance difference: {distance_error:.3f} m")

# --------------------------------
# 10. POSITION ERROR
# --------------------------------

def evaluate_position_error():
    actual_position = translation_field.getSFVec3f()

    actual_dx = actual_position[0] - initial_position[0]
    actual_dy = actual_position[1] - initial_position[1]

    # Transform odometry into world coordinates
    # Assumes robot forward is local +X
    estimated_dx = (
        initial_orientation[0] * x +
        initial_orientation[1] * y
    )

    estimated_dy = (
        initial_orientation[3] * x +
        initial_orientation[4] * y
    )

    position_error = math.sqrt(
        (estimated_dx - actual_dx)**2 +
        (estimated_dy - actual_dy)**2
    )

    print("\n--- Position Error ---")
    print(
        f"Estimated world displacement: "
        f"({estimated_dx:.3f}, {estimated_dy:.3f}) m"
    )
    print(
        f"Actual world displacement: "
        f"({actual_dx:.3f}, {actual_dy:.3f}) m"
    )
    print(f"Position error: {position_error:.3f} m")

# --------------------------------
# 11. HEADING ERROR
# --------------------------------

def evaluate_heading_error():
    current_orientation = robot_node.getOrientation()

    initial_heading = math.atan2(
        initial_orientation[3],
        initial_orientation[0]
    )

    final_heading = math.atan2(
        current_orientation[3],
        current_orientation[0]
    )

    actual_change = math.atan2(
        math.sin(final_heading - initial_heading),
        math.cos(final_heading - initial_heading)
    )

    heading_error = math.atan2(
        math.sin(theta - actual_change),
        math.cos(theta - actual_change)
    )

    print("\n--- Heading Error ---")
    print(
        f"Estimated heading change: "
        f"{math.degrees(theta):.2f} degrees"
    )
    print(
        f"Actual heading change: "
        f"{math.degrees(actual_change):.2f} degrees"
    )
    print(
        f"Heading error: "
        f"{abs(math.degrees(heading_error)):.2f} degrees"
    )

# --------------------------------
# 12. EXECUTE L-SHAPED TRAJECTORY
# --------------------------------

# First forward movement
move(SPEED, SPEED, 3.0)

# Check coordinate system after moving straight
check_coordinate_system()

# Turn approximately 90 degrees
move(-SPEED, SPEED, 0.68)

# Second forward movement
move(SPEED, SPEED, 3.0)

# Stop motors
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# --------------------------------
# 13. FINAL RESULTS
# --------------------------------

print("\nFinal estimated pose:")
print(f"x = {x:.3f} m")
print(f"y = {y:.3f} m")
print(f"theta = {math.degrees(theta):.1f} degrees")

evaluate_localization()
evaluate_position_error()
evaluate_heading_error()

while robot.step(timestep) != -1:
    pass
