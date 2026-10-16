
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")

# Access wheel encoders
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")

# Enable encoder measurements
left_encoder.enable(timestep)
right_encoder.enable(timestep)

# Enable velocity control
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

SPEED = 3.0

# Function to control robot movement
def move(left_speed, right_speed, duration):
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    steps = int(duration * 1000 / timestep)

    for _ in range(steps):
        if robot.step(timestep) == -1:
            return
        left_rotation = left_encoder.getValue()
        right_rotation = right_encoder.getValue()

        print("Left:", round(left_rotation, 3),
          "Right:", round(right_rotation, 3))

# Move forward for 3 seconds
move(SPEED, SPEED, 3.0)

# Rotate approximately 90 degrees
move(-SPEED, SPEED, 0.68)

# Move forward for another 3 seconds
move(SPEED, SPEED, 3.0)

# Stop the robot
left_motor.setVelocity(0)
right_motor.setVelocity(0)

while robot.step(timestep) != -1:
    pass
