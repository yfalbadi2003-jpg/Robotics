
from controller import Supervisor
import math
import random
import csv


# ============================================================
# 1. ROBOT SETUP
# ============================================================

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

left_motor.setPosition(float("inf"))
right_motor.setPosition(float("inf"))

# Wheel encoders
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

left_encoder.enable(timestep)
right_encoder.enable(timestep)

# Ground truth (evaluation only)
robot_node = robot.getSelf()
translation_field = robot_node.getField("translation")


# ============================================================
# 2. ROBOT PARAMETERS AND UNCERTAINTY
# ============================================================

WHEEL_RADIUS = 0.0205       # metres
WHEEL_DISTANCE = 0.052      # metres
SPEED = 3.0                 # rad/s

# Standard deviation of simulated encoder noise
ENCODER_NOISE_STD = 0.02    # radians per simulation step

# Reproducible random noise
random.seed(42)


# ============================================================
# 3. INITIALIZATION
# ============================================================

# Estimated pose in the robot's initial coordinate frame
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

# Store trajectory measurements
trajectory_data = []

print("Initial position:", initial_position)
print("Initial orientation:", initial_orientation)


# ============================================================
# 4. ODOMETRY LOCALIZATION
# ============================================================

def update_odometry():
    global x, y, theta
    global previous_left, previous_right

    # Read wheel encoders
    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    # Wheel rotation increments
    delta_left = current_left - previous_left
    delta_right = current_right - previous_right

    # Simulate encoder measurement uncertainty
    noisy_left = delta_left + random.gauss(0, ENCODER_NOISE_STD)
    noisy_right = delta_right + random.gauss(0, ENCODER_NOISE_STD)

    # Convert wheel rotations into travel distances
    distance_left = WHEEL_RADIUS * noisy_left
    distance_right = WHEEL_RADIUS * noisy_right

    # Differential-drive odometry
    delta_distance = (distance_left + distance_right) / 2
    delta_theta = (distance_right - distance_left) / WHEEL_DISTANCE

    # Update estimated pose
    x += delta_distance * math.cos(theta + delta_theta / 2)
    y += delta_distance * math.sin(theta + delta_theta / 2)
    theta += delta_theta

    # Store actual encoder readings for the next step
    previous_left = current_left
    previous_right = current_right


# ============================================================
# 5. TRAJECTORY RECORDING
# ============================================================

def record_trajectory():
    # Read simulator ground truth
    actual_position = translation_field.getSFVec3f()

    actual_dx = actual_position[0] - initial_position[0]
    actual_dy = actual_position[1] - initial_position[1]

    # Convert estimated pose into world coordinates
    estimated_dx = (
        initial_orientation[0] * x
        + initial_orientation[1] * y
    )

    estimated_dy = (
        initial_orientation[3] * x
        + initial_orientation[4] * y
    )

    # Record estimated and actual positions
    trajectory_data.append([
        estimated_dx,
        estimated_dy,
        actual_dx,
        actual_dy
    ])


def save_trajectory():
    with open("trajectory.csv", "w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow([
            "estimated_x",
            "estimated_y",
            "actual_x",
            "actual_y"
        ])

        writer.writerows(trajectory_data)

    print("\nTrajectory saved to trajectory.csv")


# ============================================================
# 6. ROBOT MOVEMENT
# ============================================================

def move(left_speed, right_speed, duration):
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    steps = int(duration * 1000 / timestep)

    for _ in range(steps):
        if robot.step(timestep) == -1:
            return

        update_odometry()
        record_trajectory()

    print(
        f"Estimated pose: "
        f"x={x:.3f} m, "
        f"y={y:.3f} m, "
        f"theta={math.degrees(theta):.1f} deg"
    )


# ============================================================
# 7. COORDINATE SYSTEM CHECK
# ============================================================

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


# ============================================================
# 8. LOCALIZATION EVALUATION
# ============================================================

def evaluate_localization():
    actual_position = translation_field.getSFVec3f()

    actual_dx = actual_position[0] - initial_position[0]
    actual_dy = actual_position[1] - initial_position[1]

    estimated_distance = math.hypot(x, y)
    actual_distance = math.hypot(actual_dx, actual_dy)

    distance_error = abs(estimated_distance - actual_distance)

    print("\n--- Localization Evaluation ---")
    print(f"Estimated displacement: {estimated_distance:.3f} m")
    print(f"Actual displacement: {actual_distance:.3f} m")
    print(f"Displacement difference: {distance_error:.3f} m")


def evaluate_position_error():
    actual_position = translation_field.getSFVec3f()

    actual_dx = actual_position[0] - initial_position[0]
    actual_dy = actual_position[1] - initial_position[1]

    estimated_dx = (
        initial_orientation[0] * x
        + initial_orientation[1] * y
    )

    estimated_dy = (
        initial_orientation[3] * x
        + initial_orientation[4] * y
    )

    position_error = math.hypot(
        estimated_dx - actual_dx,
        estimated_dy - actual_dy
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


# ============================================================
# 9. EXECUTE L-SHAPED TRAJECTORY
# ============================================================

# Record initial position
record_trajectory()

# Move forward
move(SPEED, SPEED, 3.0)

# Verify coordinate system
check_coordinate_system()

# Turn approximately 90 degrees
move(-SPEED, SPEED, 0.68)

# Move forward again
move(SPEED, SPEED, 3.0)

# Stop motors
left_motor.setVelocity(0)
right_motor.setVelocity(0)


# ============================================================
# 10. FINAL RESULTS
# ============================================================

print("\nFinal estimated pose:")
print(f"x = {x:.3f} m")
print(f"y = {y:.3f} m")
print(f"theta = {math.degrees(theta):.1f} degrees")

evaluate_localization()
evaluate_position_error()
evaluate_heading_error()

# Save trajectory data for plotting
save_trajectory()


# ============================================================
# 11. KEEP SIMULATION RUNNING
# ============================================================

while robot.step(timestep) != -1:
    pass
