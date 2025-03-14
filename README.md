# FRC Team 2386 Trojans 2025 Robot

This repository contains the code for the FRC Team Trojans robot for the 2025 season. The robot is built on the Command-Based programming paradigm using WPILib and features a swerve drivetrain with various scoring mechanisms.

## Table of Contents
- [Robot Overview](#robot-overview)
- [Subsystems](#subsystems)
  - [Drivetrain](#drivetrain)
  - [Elevator](#elevator)
  - [Harpoon](#harpoon)
  - [Climber](#climber)
  - [LED](#led)
  - [PhotonVision](#photonvision)
- [Commands](#commands)
  - [Drive Commands](#drive-commands)
  - [Elevator Commands](#elevator-commands)
  - [Harpoon Commands](#harpoon-commands)
  - [Vision Commands](#vision-commands)
- [Utilities](#utilities)
- [Controller Setup](#controller-setup)
  - [Driver Controller (Xbox)](#driver-controller-xbox)
  - [Driver Controller (PS5)](#driver-controller-ps5)
  - [Operator Controller (Xbox)](#operator-controller-xbox)
  - [Operator Controller (PS5)](#operator-controller-ps5)
- [Autonomous Operation](#autonomous-operation)
- [Development Guide](#development-guide)

## Robot Overview

The Trojans 2025 robot features a swerve drive base with several mechanisms designed for game piece manipulation and scoring:

- **Swerve Drivetrain**: REV MAXSwerve modules for precise omnidirectional movement
- **Elevator**: Vertically movable mechanism for reaching different scoring heights
- **Harpoon**: Combined intake and outtake system with rotation capability
- **Climber**: Mechanism for climbing at the end of the match
- **Vision System**: PhotonVision pose estimation for accurate field positioning

## Subsystems

### Drivetrain

The drivetrain is a swerve drive system built with REV MAXSwerve modules. It features four independently controlled modules, each with a driving and turning motor.

#### Key Features:
- Four REV MAXSwerve modules (front left, front right, rear left, rear right)
- NavX gyroscope for field-oriented control
- Odometry for position tracking
- Multiple drive modes:
  - MANUAL: Standard driver control
  - ALIGNTELE: Vision-assisted alignment
  - HEADINGLOCK: Maintains robot heading
  - AUTO: Autonomous control

#### Configuration:
- Normal Speed: 2 m/s
- Fast Speed: 3.3 m/s
- Slow Speed: 0.9 m/s
- Maximum Angular Speed: 2π radians/second

### Elevator

The elevator subsystem controls the vertical positioning of the robot's scoring mechanism.

#### Key Features:
- Dual motor system (primaryMotor and followerMotor)
- Position control with PID
- Position presets for different scoring levels (L1, L2, L3, L4)
- Built-in soft limits and calibration routine
- Three operational modes:
  - STOWED: Retracted position
  - REEF: For scoring in the reef (lower goals)
  - FEEDER: For scoring in the feeder station

#### Elevator Positions:
- STOWED: 0 rotations
- L1: 4 rotations
- L2: 8.85 rotations
- L3: 12.95 rotations
- L4: 20.3 rotations
- INTAKE: 2.65 rotations
- ALGAE: 8.55 rotations

### Harpoon

The Harpoon subsystem manages the intake/outtake mechanism and wrist rotation.

#### Key Features:
- Intake/outtake motor for game piece manipulation
- Rotation motor for wrist positioning
- Position control with PID for accurate placement
- Game piece detection
- Three operational modes:
  - STOWED: Retracted position
  - REEF: For scoring in the reef (lower goals)
  - FEEDER: For scoring in the feeder station

#### Harpoon Positions:
- L1: 0.725
- L2: 0.6003
- L3: 0.6002
- L4: 0.6001
- INTAKE: 0.95
- STOWED: 1.015
- ALGAE: 0.61

### Climber

A simple climber mechanism for end-game climbing.

#### Key Features:
- Single motor design
- Position-based control
- Preset positions for climbing sequence

### LED

Simple LED control subsystem for driver feedback.

#### Key Features:
- Dual LED strips (left and right)
- Visual feedback for various robot states and operations

### PhotonVision

Vision-based pose estimation using PhotonVision and AprilTags.

#### Key Features:
- Multiple camera support
- AprilTag detection for field positioning
- Integration with odometry for improved pose estimation
- Vision-assisted targeting and alignment

## Commands

### Drive Commands

Located in `src/main/java/frc/robot/commands/drive/`

- **TeleopDriveCommand**: Primary driving command during teleop
  - Handles joystick inputs for translation and rotation
  - Supports field-oriented driving
  - Implements rate limiting for smooth driving

### Elevator Commands

Located in `src/main/java/frc/robot/commands/elevator/`

- **CalibrateElevator**: Calibrates the elevator by finding the bottom limit
- **MoveElevatorCommand**: Moves the elevator to a specified position
- **PrepareElevatorCommand**: Sets up the elevator for a specific operation

### Harpoon Commands

Located in `src/main/java/frc/robot/commands/harpoon/`

- **AutoScoreCommand**: Automatically scores game pieces
- **IntakeCommand**: Controls the intake operation
- **EnumeratedIntakeCommand**: Specialized intake command with additional parameters
- **PrepareHarpoonCommand**: Prepares the harpoon for a specific operation
- **ShootCommand**: Controls the shooting/scoring operation
- **SequentialScoreCommand**: Sequential operation for scoring
- **StopClawCommand**: Stops the claw operation

### Vision Commands

Located in `src/main/java/frc/robot/commands/vision/`

- **AlignTeleopCommand**: Uses vision to align the robot during teleop

## Utilities

Located in `src/main/java/frc/utils/`

- **SwerveUtils**: Utility methods for swerve drive calculations
- **VisionUtils**: Utility methods for vision processing
- **AutoUtils**: Utility methods for autonomous operation
- **CameraTransform**: Manages camera transformation data
- **TagTransform**: Manages AprilTag transformation data

## Controller Setup

The robot supports both Xbox and PS5 controllers, which can be selected via SmartDashboard. The controller type can be selected in the dashboard using the "Driver Select" and "Operator Select" dropdown menus.

### Driver Controller (Xbox)

#### Movement Controls:
- **Left Joystick**: Translation (forward/backward, left/right)
- **Right Joystick**: Rotation
- **Left Bumper**: Enable slow mode (0.9 m/s) while held
- **Right Bumper**: Enable fast mode (3.3 m/s) while held

#### Function Buttons:
- **Y Button**: Reset gyro (zero heading)
- **A Button**: Empty command (placeholder)
- **X Button**: Intake game piece
- **B Button**: Shoot/outtake game piece (stops when released)
- **D-Pad Down**: Calibrate elevator

### Driver Controller (PS5)

#### Movement Controls:
- **Left Joystick**: Translation (forward/backward, left/right)
- **Right Joystick**: Rotation
- **R1 Button**: Enable fast mode (3.3 m/s) while held

#### Function Buttons:
- **L1 Button**: Reset gyro (zero heading)
- **Square Button**: Intake game piece (enumerated intake)
- **Circle Button**: Shoot/outtake game piece (stops when released)
- **Triangle Button**: Calibrate elevator
- **R2 Button**: Align to right-side scoring position
- **L2 Button**: Align to left-side scoring position
- **D-Pad Up**: Score at L4 position
- **D-Pad Right**: Score at L1 position
- **D-Pad Down**: Score at L2 position
- **D-Pad Left**: Score at L3 position

### Operator Controller (Xbox)

The operator controller manages mechanisms for scoring and manipulation.

#### Function Buttons:
- **Left Bumper**: Prepare elevator and harpoon for lower reef scoring
- **Right Bumper**: Prepare elevator and harpoon for upper reef scoring
- **Y Button**: Prepare elevator and harpoon for intake position

### Operator Controller (PS5)

#### Function Buttons:
- **L1 Button**: Prepare elevator and harpoon for lower reef scoring
- **R1 Button**: Prepare elevator and harpoon for upper reef scoring
- **Triangle Button**: Prepare elevator and harpoon for intake position

## Autonomous Operation

The robot uses PathPlanner for autonomous path planning and execution.

### Features:
- Multiple autonomous routines available
- Path selection via SmartDashboard
- Alliance-specific path mirroring
- Integrated with vision for improved accuracy

### Available Auto Routines:
- Various path combinations for different starting positions and strategies
- Configured in Constants.AutoStrings

## Development Guide

### Prerequisites
- WPILib 2024 or later
- Visual Studio Code with WPILib extension
- REV Hardware Client for motor configuration
- Git for version control

### Building and Deploying
1. Clone the repository
   ```
   git clone https://github.com/yourusername/Trojans2025.git
   ```
2. Open the project in VS Code
3. Build the project using the WPILib build command
4. Deploy to the robot using the WPILib deploy command

### Code Structure
- **Robot.java**: The main robot class
- **RobotContainer.java**: Contains subsystem instantiation and button bindings
- **Constants.java**: All robot constants and configuration values
- **Subsystems/**: All robot subsystems
- **Commands/**: All robot commands
- **Utils/**: Utility classes

### Adding New Features
1. Create new subsystems in the Subsystems directory
2. Create new commands in the Commands directory
3. Add necessary constants to Constants.java
4. Configure button bindings in RobotContainer.java

### Testing
- Use simulation for initial testing when possible
- Test each subsystem individually before integration
- Use SmartDashboard/Shuffleboard to monitor robot state during testing

## License

This project is licensed under the WPILib BSD License - see the WPILib-License.md file for details. 