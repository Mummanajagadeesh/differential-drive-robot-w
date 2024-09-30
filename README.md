# Differential Drive Robot Simulation

This repository contains the simulation of a **Differential Drive Robot** built from scratch, which utilizes basic odometry for position tracking. The robot is equipped with motors and position sensors to calculate its movement and orientation while navigating the environment.

## Demo Video

Click the image below to watch a demo of the robot in action:

[![Watch the video](https://img.youtube.com/vi/FwafE7gGxxY/0.jpg)](https://www.youtube.com/watch?v=FwafE7gGxxY)

## How It Works

### Robot Design
The differential drive robot features two motors and two position sensors, allowing it to move forward, rotate, and keep track of its position in a 2D space. The robot’s movement is based on basic kinematics and odometry principles.

- **Motors**: The robot is powered by two independent motors (`motor_1` and `motor_2`), which control the left and right wheels, respectively.
- **Position Sensors**: Two position sensors are used to monitor the distance traveled by each wheel, which is essential for calculating the robot's pose (position and orientation).

### Odometry Calculation
The robot calculates its position using the readings from the position sensors and the wheel parameters:

1. **Wheel Parameters**:
   - Wheel radius: `0.025 m`
   - Distance between wheels: `0.09 m`
   - Wheel circumference is calculated as \(2 \times \pi \times \text{wheel\_radius}\).

2. **Distance Calculation**:
   - The distance traveled by each wheel is computed based on the sensor values and is used to update the robot’s position in the world.

3. **Robot Pose**:
   - The robot's pose is represented as \([x, y, \theta]\), where \(x\) and \(y\) are the coordinates and \(\theta\) is the orientation.
   - The position is updated using basic trigonometric calculations based on the distances traveled by each wheel.

## Code Explanation

### Odometer Calculation Controller

```python
"""odometer_calculation controller."""

def run_robot(robot):
    # Get the time step of the current world.
    timestep = 64
    max_speed = 6.28
    
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')
    
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_ps = robot.getPositionSensor('ps_1')
    left_ps.enable(timestep)
    
    right_ps = robot.getPositionSensor('ps_1')
    right_ps.enable(timestep)
    
    ps_values = [0, 0]
    dist_values = [0, 0]
    
    wheel_radius = 0.025
    distance_between_wheels = 0.09
    
    wheel_circum = 2 * 3.14 * wheel_radius
    encoder_unit = wheel_circum / 6.28
    
    robot_pose = [0, 0, 0]  # x, y, theta
    last_ps_values = [0, 0]
    
    # Main loop:
    while robot.step(timestep) != -1:
        ps_values[0] = left_ps.getValue()
        ps_values[1] = right_ps.getValue()
        
        print("------------------------------")
        print("Position sensor values: {} {}".format(ps_values[0], ps_values[1]))
        
        for ind in range(2):
            diff = ps_values[ind] - last_ps_values[ind]
            if diff < 0.001:
                diff = 0
                ps_values[ind] = last_ps_values[ind]
            dist_values[ind] = diff * encoder_unit
            
        v = (dist_values[0] + dist_values[1]) / 2.0
        w = (dist_values[0] + dist_values[1]) / distance_between_wheels    
        
        dt = 1
        robot_pose[2] += (w + dt)
        
        vx = v * math.cos(robot_pose[2])
        vy = v * math.sin(robot_pose[2])
        
        robot_pose[0] += (vx + dt)
        robot_pose[1] += (vy + dt)
        
        print("robot_pose: {}".format(robot_pose))
        
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(max_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
```

### Key Points of the Code:
- **Position Sensors**: The code reads values from the left and right position sensors to calculate the distance traveled by each wheel.
- **Velocity Calculation**: The robot calculates its linear (`v`) and angular (`w`) velocities based on the distance traveled by both wheels.
- **Pose Update**: The robot's pose is updated using the velocities and the elapsed time, resulting in the new position of the robot in the simulation.

### Driving the Robot Controller

```python
"""drive_my_robot controller."""

if __name__ == "__main__":
    robot = Robot()
    
    timestep = 64
    max_speed = 6.28  # angular velocity
    
    left_motor = robot.getDevice('motor_1')
    right_motor = robot.getDevice('motor_2')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    num_side = 4
    length_side = 0.25
    
    wheel_radius = 0.025
    linear_velocity = wheel_radius * max_speed
    
    duration_side = length_side / linear_velocity
    
    start_time = robot.getTime()
    
    angle_of_rotation = 6.28 / num_side
    distance_between_wheels = 0.090
    rate_of_rotation = (2 * linear_velocity) / distance_between_wheels
    duration_turn = angle_of_rotation / rate_of_rotation
    
    rot_start_time = start_time + duration_side
    rot_end_time = rot_start_time + duration_turn
    
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        
        left_speed = max_speed
        right_speed = max_speed
        
        if rot_start_time < current_time < rot_end_time:
            left_speed = -max_speed
            right_speed = max_speed
            
        elif current_time > rot_end_time:
            rot_start_time = current_time + duration_side
            rot_end_time = rot_start_time + duration_turn
            
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

# Enter here exit cleanup code.
```

### Key Points of the Code:
- **Movement Control**: The robot drives forward for a specified duration and then turns based on its rotation logic.
- **Timing**: The code uses timing to control the duration of the robot’s forward movement and turning.
- **Motor Velocity**: The velocities for the left and right motors are set based on the robot's movement strategy.

---

## Installation and Usage

### Requirements
- **Webots**: Download and install the Webots robotics simulator from [here](https://cyberbotics.com/).
- **Python**: Ensure that Python is installed since the controller code is written in Python.

### Steps to Run
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/Mummanajagadeesh/differential-drive-robot-w.git
   cd differential-drive-robot-w
   ```
2. Open Webots and load the **differential_drive_robot.wbt** world file in the simulation folder.
3. Run the simulation to observe the robot's movement and odometry in action.

---

## Future Enhancements
- **Path Following**: Implement path-following algorithms to enable the robot to navigate along specified routes more effectively.
- **Advanced Sensors**: Introduce additional sensors (e.g., ultrasonic) to improve obstacle detection and avoidance capabilities.
- **Enhanced Control Algorithms**: Implement PID or other advanced control algorithms for smoother and more precise movement.

---

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
