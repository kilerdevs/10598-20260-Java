# FRC Team 10598 — 2026 Competition Robot

**Java · WPILib 2026 · Command-Based · AdvantageKit · Phoenix 6 · REVLib · PathPlanner · PhotonVision**

Competition robot code for FRC Team 10598's 2026 season. Implements a full ball-handling pipeline — intake → spindexer → feeder → flywheel shooter — with a motorized turret, automated field-relative aim assist, dual-camera AprilTag pose estimation, and replay-capable structured logging.

---

## Robot Overview

The robot is built around a **swerve drivetrain** with a **shooting pipeline** that can score game pieces while the robot is in motion. Key systems:

| System | Mechanism | Actuators |
|---|---|---|
| **Drivetrain** | 4-module swerve (Kraken X60) | 4× TalonFX (drive) + 4× TalonFX (steer) |
| **Intake** | Pivoting arm + compliant-wheel roller | 2× NEO 550 (pivot) + 1× NEO (roller) |
| **Spindexer** | Rotary ball indexer | 1× NEO |
| **Feeder** | Belt feeder to shooter | 1× NEO 1.1 |
| **Shooter** | Flywheel + adjustable hood + turret | 2× NEO (flywheel) + 1× NEO (hood) + 1× NEO (turret) |
| **LEDs** | 49-LED CANdle strip | CANdle (CAN 14) |

---

## Architecture

```
src/main/java/frc/robot/
├── subsystems/
│   ├── drive/           # Swerve drive (IO abstraction layer)
│   │   ├── Drive.java
│   │   ├── Module.java
│   │   ├── ModuleIO.java          # Interface → real / sim
│   │   ├── ModuleIOTalonFX.java
│   │   ├── ModuleIOTalonFXSim.java
│   │   ├── GyroIO.java            # Interface → Pigeon 2 / NavX / sim
│   │   ├── GyroIOPigeon2.java
│   │   └── PhoenixOdometryThread.java
│   ├── shooter/         # Flywheel, hood, turret
│   ├── intake/          # Pivot arm state machine
│   ├── spindexer/       # Ball indexer + feeder
│   ├── vision/          # Dual PhotonVision cameras
│   ├── leds/            # CANdle LED feedback
│   ├── gamepad/         # Rumble management
│   └── zones/           # Field-zone polygon triggers
├── commands/
│   └── DriveCommands.java   # Joystick drive, auto-aim, characterization
├── util/
│   └── LocalADStarAK.java   # Custom A* path-planning adapter
├── generated/
│   └── TunerConstants.java  # Phoenix 6 Tuner-generated swerve config
├── RobotContainer.java      # Subsystem wiring, button bindings
├── Robot.java               # LoggedRobot entry point (AdvantageKit)
├── Constants.java           # Runtime mode (real / sim / replay)
└── BuildConstants.java      # Auto-generated Git metadata
```

---

## Sophisticated Features

### 1. AdvantageKit Structured Logging with Replay

The robot runs on **AdvantageKit** (`LoggedRobot`), a framework developed by FRC 6328 Mechanical Advantage that records every subsystem input and output to a `.wpilog` file. Logs can be replayed deterministically in simulation — the exact same code that ran on the field re-executes against the recorded sensor data, allowing post-match debugging without touching hardware.

- All subsystem I/O is declared via `@AutoLog`-annotated interfaces
- Build metadata (Git SHA, branch, dirty status, build date) is injected at compile time via `BuildConstants.java` and logged on startup
- NetworkTables publisher runs in parallel for real-time dashboards

### 2. IO Abstraction Layer

Every hardware-touching subsystem is split into an **interface + implementation** pattern:

```
GyroIO          → GyroIOPigeon2  (real)
                → GyroIONavX     (fallback)
                → GyroIOSim      (simulation)

ModuleIO        → ModuleIOTalonFX       (real)
                → ModuleIOTalonFXSim    (Phoenix sim)
                → ModuleIOSim           (pure WPILib sim)

VisionIO        → VisionIOPhotonVision     (real)
                → VisionIOPhotonVisionSim  (simulation)
```

The same `Robot.java` runs unmodified in simulation, log replay, or on the physical robot — only the IO implementations change. This enables continuous integration testing and practice driving without hardware.

### 3. Swerve Drivetrain (Phoenix 6 + CANivore)

All drivetrain devices communicate over a **CANivore** (CAN FD) bus named `"miesozerca"`, allowing 250 Hz odometry updates compared to 100 Hz on standard CAN 2.0.

- **8× CTRE Kraken X60** motors (4 drive + 4 steer) via `TalonFX`
- **4× CANcoder** absolute encoders for steer angle
- **Pigeon 2.0** IMU for gyro heading
- **Phoenix 6 odometry thread** (`PhoenixOdometryThread.java`) runs independently of the main robot loop, synchronising timestamps from all four modules and accumulating pose at up to 250 Hz
- Field-relative control with automatic alliance-based heading flip
- Configurable soft slow-mode (40% speed) for congested field areas
- **MapleSim** physics engine integration for realistic simulation (74 kg robot mass, 6.88 kg·m² MOI, 1.2 wheel COF)

Swerve module parameters (from Tuner X):

| Parameter | Value |
|---|---|
| Wheel radius | 1.89 in |
| Drive gear ratio | 5.9 : 1 |
| Steer gear ratio | 18.75 : 1 |
| Coupling ratio | 3.125 : 1 |
| Wheelbase | 10.75 × 10.75 in |
| Drive slip current | 120 A |
| Max linear speed @ 12 V | 14.23 m/s |

### 4. Dual-Camera AprilTag Pose Estimation

Two **PhotonVision** cameras fuse their observations into a single `SwerveDrivePoseEstimator`:

| Camera | Position (robot-relative) | Pitch | Yaw |
|---|---|---|---|
| FR_Camera | −0.272 m fwd, −0.272 m left, 0.192 m up | 15° | 180° |
| BR_Camera | +0.253 m fwd, −0.255 m left, 0.192 m up | 15° | −90° |

Observations are filtered before being fed to the estimator:

- **Distance filter**: reject tags more than 7.0 m away
- **Ambiguity filter**: reject poses with ambiguity > 0.2
- **Z-error filter**: reject if estimated Z deviates > 0.75 m from field floor
- **Standard deviation scaling**: confidence decreases with distance and increases with tag count; MegaTag 2 multi-tag observations receive a 0.5× multiplier on linear std dev and infinite angular std dev (heading locked to gyro)

The estimator continuously corrects dead-reckoning drift between vision updates, enabling accurate autonomous path following and turret auto-aim.

### 5. Distance-Based Shooter Auto-Aim

The shooter has three motorised degrees of freedom: **flywheel speed**, **hood angle**, and **turret heading**. All three are controlled automatically based on real-time distance to the scoring hub.

**Shot map interpolation** (`deploy/shotmap.json`):

```
1.0 m → 2500 RPM @ 50.0°
2.0 m → 3200 RPM @ 51.0°
3.0 m → 4000 RPM @ 54.0°
4.0 m → 4600 RPM @ 56.5°
5.0 m → 5200 RPM @ 59.5°
6.0 m → 5500 RPM @ 61.5°
```

Linear interpolation between entries runs every 20 ms. Hub positions are hard-coded per alliance:

- **Blue alliance hub**: (4.57 m, 4.0 m)
- **Red alliance hub**: (11.97 m, 4.0 m)

**Turret heading** is computed from the field-relative robot pose and hub position, then compensated by the current gyro heading to produce a robot-relative setpoint. The turret has a REV SparkMax PID position controller with soft limits (−34.2 → +6.4 encoder rotations, mapping to ≈ 40° of travel at 37.5 : 1 gear ratio).

**Hood angle** uses a separate REV SparkMax position controller with soft limits at 45° and 65°. Fine trim is available via D-pad (±1.2° per 20 ms).

**Flywheel** uses velocity PID with a feedforward tuned to the NEO's free-speed RPM:

```
KFF = 1 / 5676   (1 / NEO max RPM)
Ki integral zone = 500 RPM
Ki accumulator cap = 0.15
```

### 6. Intake State Machine with Auto-Calibration

The intake pivot uses **6 discrete states** and calibrates itself on first enable without any absolute encoder:

```
WAITING_FOR_ENABLE
    → CALIBRATING        (slow upward drive until current spike > 5 A)
    → UPPER              (known upper hardstop)
    → MOVING_TO_LOWER
    → LOWER              (current-spike detection at lower hardstop)
    → MOVING_TO_UPPER
```

**MAXMotion** (REV's motion profile controller) drives the pivot between upper and lower positions with a configurable cruise velocity (2000 RPM) and acceleration (1500 RPM/s). The lower position is detected by monitoring motor current (threshold: 15 A) and velocity (< 50 RPM) — no limit switch required.

The roller motor (30 A current-limited NEO) runs only when the intake is deployed and not suppressed by the shooter pipeline.

### 7. Shooter Hood Auto-Calibration

The hood angle motor also self-calibrates without an absolute sensor using a **4-state seeking routine**:

```
WAITING_FOR_CALIBRATION
    → SEEKING_UPPER_HARDSTOP   (drives until velocity < 20 RPM, current > 3 A)
    → SETTLING                 (300 ms pause for mechanical settling)
    → SEEKING_LOWER_HARDSTOP   (drives to lower hardstop, zeroes encoder)
    → CALIBRATED               (soft limits active, PID control enabled)
```

After calibration the encoder is zeroed at the lower hardstop and soft limits are armed, preventing mechanical over-travel.

### 8. Feeder Readiness Check

The feeder motor (NEO 1.1) must reach **90% of its no-load RPM** before the robot reports `isReadyToShoot()`:

```java
// Threshold: 90% of 5880 RPM (NEO 1.1 max)
private static final double FEEDER_READY_RPM = 5880 * 0.9; // = 5292 RPM
```

The LED strip and gamepad rumble reflect shooter readiness in real time.

### 9. Field-Zone Polygon Triggers

Field zones are defined as polygons in `deploy/zones.json` using field coordinates:

| Zone | Name | Action |
|---|---|---|
| Zone 0 | PREPARE_SHOOT | Spin up shooter on zone entry |
| Zone 1–2 | AUTO_INTAKE | Deploy intake on zone entry |
| Zone 3 | AIM_AT_TARGET | Enable turret auto-aim |

Point-in-polygon detection uses a **ray-casting algorithm** (`ZoneActionSubsystem.java`). All coordinates are automatically mirrored for Red alliance. Hub distance is recalculated every 20 ms from the estimated pose.

### 10. PathPlanner Autonomous with A* Pathfinding

Autonomous paths are created in **PathPlanner** and stored as JSON in `deploy/pathplanner/`. Three pre-built autos ship with the code: `fjut`, `LeftSide`, and `RightSide`.

- `PPHolonomicDriveController` provides closed-loop path following with independent translation and rotation PID
- Default path constraints: 5.5 m/s, 5.5 m/s², 540°/s, 720°/s²
- A custom `LocalADStarAK.java` wraps PathPlanner's A\* pathfinder to be compatible with AdvantageKit's logging layer
- Starting pose is reset automatically from the first path waypoint at auto init
- Characterisation routines (feedforward ramp, wheel radius, SysId quasistatic/dynamic) are available as selectable auto commands for on-field tuning

### 11. LED State Machine (13 States)

The CANdle drives a 49-LED GRB strip with distinct animations per robot state:

| State | Animation |
|---|---|
| IDLE | Larson scanner (knight-rider) |
| INTAKING | Strobe green |
| SHOOTING | Strobe orange |
| HAS_GAME_PIECE | Solid green |
| ALIGNING / TAG_TRACKING | Solid blue |
| TAG_LOST | Strobe red |
| READY | Solid green pulse |
| NOT_READY | Solid red |
| AUTO | Larson scanner purple |
| DISABLED | Fade blue |
| ERROR | Strobe red |
| UPLOADING | Fade yellow |

### 12. Gamepad Rumble Management

`GamepadSubsystem` tracks independent left/right rumble motors with **expiry timestamps** — each rumble call specifies intensity and duration without blocking the command scheduler. The rumble controller automatically stops each motor when its timestamp expires.

### 13. Drive Characterisation Tools

The `DriveCommands.java` command library includes first-principles characterisation routines that can be selected from the auto chooser:

- **Feedforward characterisation**: ramps voltage from 0 → 12 V, records velocity, fits `Ks` and `Kv`
- **Wheel radius characterisation**: rotates robot in place, integrates wheel arc vs gyro angle to compute actual wheel radius (accounts for wear)
- **WPILib SysId integration**: quasistatic and dynamic tests for full `Ks`, `Kv`, `Ka` identification

---

## Technology Stack

| Category | Technology |
|---|---|
| Language | Java 17 |
| Build system | Gradle + GradleRIO 2026.2.1 |
| Robot framework | WPILib 2026, Command-Based |
| Logging | AdvantageKit (6328 Mechanical Advantage) |
| Drive hardware | CTRE Phoenix 6 — TalonFX, CANcoder, Pigeon 2.0, CANdle |
| Other motors | REV Robotics SparkMax + NEO / NEO 1.1 / NEO 550 |
| Vision | PhotonVision + AprilTag field layout |
| Autonomous | PathPlanner 2026.1.2 |
| Simulation | IronMaple MapleSim (swerve physics) |
| Gyro (backup) | navX2 (Studica) |
| CAN bus | CANivore (CAN FD, 250 Hz odometry) |

---

## Building & Deploying

**Requirements**: Java 17 JDK, WPILib 2026 installation

```bash
# Build (no robot needed)
./gradlew build

# Deploy to robot over USB or network
./gradlew deploy

# Run simulation
./gradlew simulateJava
```

The Gradle build embeds a source backup, vendordep JSONs, and `build.gradle` inside the deployed JAR so the exact code that produced any log file can always be recovered.

---

## Repository

**Team**: FRC 10598  
**Season**: 2026  
**Language**: Java  
**Robot**: Event robot (full competition spec)
