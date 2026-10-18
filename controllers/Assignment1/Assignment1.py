
from controller import Supervisor
import math

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

# Ground truth (used only for evaluation)
robot_node = robot.getSelf()
translation_field = robot_node.getField("translation")

# Approximate e-puck dimensions (metres)
WHEEL_RADIUS = 0.0205
WHEEL_DISTANCE = 0.052

SPEED = 3.0

# Initial estimated pose
x = 0.0
y = 0.0
theta = 0.0

# Allow encoders to initialize
robot.step(timestep)

previous_left = left_encoder.getValue()
previous_right = right_encoder.getValue()

# Initial ground-truth position
initial_position = translation_field.getSFVec3f()


def update_odometry():
    global x, y, theta, previous_left, previous_right

    # Read current wheel angles
    current_left = left_encoder.getValue()
    current_right = right_encoder.getValue()

    # Calculate change in wheel rotation
    delta_left = current_left - previous_left
    delta_right = current_right - previous_right

    # Convert wheel rotation to distance
    distance_left = WHEEL_RADIUS * delta_left
    distance_right = WHEEL_RADIUS * delta_right

    # Calculate forward movement and heading change
    delta_distance = (distance_left + distance_right) / 2
    delta_theta = (distance_right - distance_left) / WHEEL_DISTANCE

    # Update estimated position
    x += delta_distance * math.cos(theta + delta_theta / 2)
    y += delta_distance * math.sin(theta + delta_theta / 2)
    theta += delta_theta

    # Store encoder readings for next step
    previous_left = current_left
    previous_right = current_right

def evaluate_localization():
    actual_position = translation_field.getSFVec3f()

    # Actual displacement from starting position
    actual_dx = actual_position[0] - initial_position[0]
    actual_dz = actual_position[2] - initial_position[2]

    # Ground-truth displacement in Webots coordinates
    actual_distance = math.sqrt(actual_dx**2 + actual_dz**2)

    # Odometry displacement
    estimated_distance = math.sqrt(x**2 + y**2)

    # Compare distances from the starting point
    distance_error = abs(estimated_distance - actual_distance)

    print("\n--- Localization Evaluation ---")
    print(f"Estimated distance: {estimated_distance:.3f} m")
    print(f"Actual distance: {actual_distance:.3f} m")
    print(f"Distance difference: {distance_error:.3f} m")

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
        f"y={y:.3f} m, theta={math.degrees(theta):.1f} deg"
    )


# L-shaped trajectory
move(SPEED, SPEED, 3.0)
move(-SPEED, SPEED, 0.68)
move(SPEED, SPEED, 3.0)

# Stop
left_motor.setVelocity(0)
right_motor.setVelocity(0)

print("Final estimated pose:")
print(f"x = {x:.3f} m")
print(f"y = {y:.3f} m")
print(f"theta = {math.degrees(theta):.1f} degrees")

evaluate_localization()

while robot.step(timestep) != -1:
    pass
