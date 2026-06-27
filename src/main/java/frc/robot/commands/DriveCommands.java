// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.util.FlippingUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.vision.VisionConstants;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.5; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VOLTAGE = 2.0; // Volts
  private static final double WHEEL_RADIUS_VOLTAGE_RAMP_RATE = 0.5; // Volts/Sec

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Get linear velocity
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          // Apply rotation deadband
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // Square rotation value for more precise control
          omega = Math.copySign(omega * omega, omega);
          // Convert to field relative speeds & send command
          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                  omega * drive.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Create PID controller
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              boolean isFlipped =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      speeds,
                      isFlipped
                          ? drive.getRotation().plus(new Rotation2d(Math.PI))
                          : drive.getRotation()));
            },
            drive)

        // Reset PID controller when command starts
        .beforeStarting(() -> angleController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Allow modules to orient
        Commands.run(
                () -> {
                  drive.runCharacterization(0.0);
                },
                drive)
            .withTimeout(FF_START_DELAY),

        // Start timer
        Commands.runOnce(timer::restart),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  drive.runCharacterization(voltage);
                  velocitySamples.add(drive.getFFCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }

  private static final double WHEEL_RADIUS_ROTATIONS_GOAL = 10.0; // full rotations before auto-stop
  private static final double WHEEL_RADIUS_PRINT_INTERVAL = 0.5; // seconds between live prints

  /** Measures the robot's wheel radius by spinning in a circle. Auto-stops after 10 rotations. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_VOLTAGE_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    Timer printTimer = new Timer();

    return Commands.parallel(
        // Drive control sequence
        Commands.sequence(
            // Reset acceleration limiter
            Commands.runOnce(
                () -> {
                  limiter.reset(0.0);
                }),

            // Turn in place using open-loop voltage, accelerating up to max voltage
            Commands.run(
                () -> {
                  double voltage = limiter.calculate(WHEEL_RADIUS_MAX_VOLTAGE);
                  drive.runRotationCharacterization(voltage);
                },
                drive)),

        // Measurement sequence
        Commands.sequence(
            // Wait for modules to fully orient before starting measurement
            Commands.waitSeconds(1.0),

            // Record starting measurement
            Commands.runOnce(
                () -> {
                  state.positions = drive.getWheelRadiusCharacterizationPositions();
                  state.lastAngle = drive.getRotation();
                  state.gyroDelta = 0.0;
                  printTimer.restart();
                }),

            // Update gyro delta and print live results
            Commands.run(
                    () -> {
                      var rotation = drive.getRotation();
                      state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                      state.lastAngle = rotation;

                      // Print live wheel radius every 0.5s once we have data
                      if (state.gyroDelta > 0.1 && printTimer.advanceIfElapsed(WHEEL_RADIUS_PRINT_INTERVAL)) {
                        double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheelDelta = 0.0;
                        for (int i = 0; i < 4; i++) {
                          wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;
                        NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println(
                            "[WheelRadius] "
                                + formatter.format(Units.radiansToDegrees(state.gyroDelta) / 360.0)
                                + " rotations | radius = "
                                + formatter.format(wheelRadius)
                                + " m ("
                                + formatter.format(Units.metersToInches(wheelRadius))
                                + " in)");
                      }
                    })
                // Auto-stop after reaching the rotation goal
                .until(
                    () -> state.gyroDelta >= WHEEL_RADIUS_ROTATIONS_GOAL * 2.0 * Math.PI)

                // When done, calculate and print final results
                .finallyDo(
                    () -> {
                      double[] positions = drive.getWheelRadiusCharacterizationPositions();
                      double wheelDelta = 0.0;
                      for (int i = 0; i < 4; i++) {
                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                      }
                      double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                      NumberFormat formatter = new DecimalFormat("#0.000");
                      System.out.println(
                          "********** Wheel Radius Characterization Results **********");
                      System.out.println(
                          "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                      System.out.println(
                          "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                      System.out.println(
                          "\tWheel Radius: "
                              + formatter.format(wheelRadius)
                              + " meters, "
                              + formatter.format(Units.metersToInches(wheelRadius))
                              + " inches");
                    })));
  }

  /**
   * Repeatedly jerks the robot in pure rotation, violently oscillating by 10 degrees.
   * Alternates between +10 deg and -10 deg from the starting heading.
   *
   * <p>Uses a timer-based flip instead of PID atSetpoint() to guarantee switching.
   * The PID setpoint is set once per flip so the controller accumulates error correctly.
   */
  @SuppressWarnings("resource")
  public static Command jerkRotation(Drive drive) {
    final double jerkAngleRad = Units.degreesToRadians(10.0);
    // Half-period: how long to drive toward each target before flipping.
    // 0.15 s is aggressive; increase to 0.25 s if the robot can't keep up.
    final double halfPeriodSecs = 0.15;

    PIDController pid = new PIDController(20.0, 0.0, 0.5);
    pid.enableContinuousInput(-Math.PI, Math.PI);

    // Mutable state captured by lambdas
    double[] targets   = new double[2]; // [0] = +10 deg target, [1] = -10 deg target
    int[]    activeIdx = {0};
    double[] flipTime  = {0.0};         // timestamp of last flip
    Timer    timer     = new Timer();

    return Commands.run(
            () -> {
              // Time to flip target?
              if (timer.get() - flipTime[0] >= halfPeriodSecs) {
                activeIdx[0] = 1 - activeIdx[0];
                flipTime[0]  = timer.get();
                // Set setpoint once so PID accumulates error from here
                pid.setSetpoint(targets[activeIdx[0]]);
              }

              double omega = pid.calculate(drive.getRotation().getRadians());
              drive.runVelocity(new ChassisSpeeds(0.0, 0.0, omega));
            },
            drive)
        .beforeStarting(
            () -> {
              double current = drive.getRotation().getRadians();
              targets[0]    = current + jerkAngleRad;
              targets[1]    = current - jerkAngleRad;
              activeIdx[0]  = 0;
              flipTime[0]   = 0.0;
              pid.reset();
              pid.setSetpoint(targets[0]);
              timer.restart();
            });
  }

  /**
   * Visual-servoing command: drives the robot to 1m directly in front of AprilTag ID 1, facing the
   * tag with zero relative rotation. Uses the PhotonVision camera directly — does NOT depend on the
   * field layout. Works with any AprilTag 1 placed anywhere.
   *
   * <p>Control is fully robot-relative: the camera sees the tag, computes where it is relative to
   * the robot, and PID drives to make the tag exactly 1m forward, centered, and facing back.
   */
  @SuppressWarnings("resource")
  public static Command driveToAprilTag1(Drive drive, LEDSubsystem leds) {
    PhotonCamera camera = new PhotonCamera(VisionConstants.camera0Name);
    Transform3d robotToCamera = VisionConstants.robotToCamera0;

    // PID controllers (robot-relative axes)
    // Forward: tag should be 1.0m ahead (tagX → 1.0)
    // Lateral: tag should be centered (tagY → 0.0)
    // Heading: tag should face back at us (relative yaw → π)
    PIDController forwardController = new PIDController(2.0, 0.0, 0.1);
    PIDController lateralController = new PIDController(2.0, 0.0, 0.1);
    PIDController headingController = new PIDController(1.0, 0.0, 0.5);
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    forwardController.setSetpoint(1.0);
    lateralController.setSetpoint(0.0);
    headingController.setSetpoint(Math.PI);

    // Finish when within 5 cm and 2°
    forwardController.setTolerance(0.05);
    lateralController.setTolerance(0.05);
    headingController.setTolerance(Units.degreesToRadians(2.0));

    // Low-pass filters to smooth noisy vision measurements (reduces jerky wheel rotation)
    // singlePoleIIR(timeConstant, period): ~0.1s smoothing at 50 Hz
    LinearFilter xFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    LinearFilter yFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    LinearFilter yawFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    // Mutable state captured by lambdas
    Transform3d[] lastCameraToTag = {null};
    double[] lastObservationTimestamp = {0.0};
    double[] lastResultTimestamp = {-1.0};
    double[] lastAmbiguity = {0.0};
    boolean[] tagVisible = {false};
    final double TAG_LOST_TIMEOUT = 0.5; // seconds without a detection before stopping

    return Commands.run(
            () -> {
              // ── Read latest PhotonVision result for tag ID 1 ─────────────
              // Use getLatestResult() instead of getAllUnreadResults() so we
              // don't compete with the Vision subsystem's queue reads.
              var result = camera.getLatestResult();
              double resultTimestamp = result.getTimestampSeconds();

              // Only process if this is a genuinely new frame
              if (resultTimestamp > lastResultTimestamp[0]) {
                lastResultTimestamp[0] = resultTimestamp;
                for (var target : result.targets) {
                  if (target.fiducialId == 1) {
                    lastCameraToTag[0] = target.bestCameraToTarget;
                    lastObservationTimestamp[0] = Timer.getFPGATimestamp();
                    lastAmbiguity[0] = target.poseAmbiguity;
                  }
                }
              }

              // Check if tag is stale or never seen
              double timeSinceLastObs =
                  Timer.getFPGATimestamp() - lastObservationTimestamp[0];
              boolean tagLost =
                  lastCameraToTag[0] == null || timeSinceLastObs > TAG_LOST_TIMEOUT;

              if (tagLost) {
                tagVisible[0] = false;
                drive.stop();
                leds.setState(LEDSubsystem.LEDState.TAG_LOST);
                Logger.recordOutput("DriveToTag/TagVisible", false);
                Logger.recordOutput("DriveToTag/TimeSinceLastObsS", timeSinceLastObs);
                return;
              }

              tagVisible[0] = true;
              leds.setState(LEDSubsystem.LEDState.TAG_TRACKING);

              // ── Compute tag pose in the robot frame ──────────────────────
              Transform3d robotToTag = robotToCamera.plus(lastCameraToTag[0]);
              double rawTagX = robotToTag.getX(); // forward
              double rawTagY = robotToTag.getY(); // left
              double tagZ = robotToTag.getZ(); // up
              double rawTagYaw = robotToTag.getRotation().getZ(); // yaw

              // ── Low-pass filter to smooth noisy measurements ─────────────
              double tagX = xFilter.calculate(rawTagX);
              double tagY = yFilter.calculate(rawTagY);
              double tagRelativeYaw = yawFilter.calculate(rawTagYaw);

              // ── PID control (outputs negated: +vx moves robot forward, ──
              // ── which DECREASES tagX, so plant gain is negative) ─────────
              double rawForward = forwardController.calculate(tagX);
              double rawLateral = lateralController.calculate(tagY);
              double rawHeading = headingController.calculate(tagRelativeYaw);

              double vx = MathUtil.clamp(-rawForward, -1.5, 1.5);
              double vy = MathUtil.clamp(-rawLateral, -1.5, 1.5);
              double omega = -rawHeading;

              drive.runVelocity(new ChassisSpeeds(vx, vy, omega));

              // ── Extensive AdvantageKit logging ───────────────────────────
              // Tag in robot frame
              Logger.recordOutput("DriveToTag/TagVisible", true);
              Logger.recordOutput("DriveToTag/TagXm", tagX);
              Logger.recordOutput("DriveToTag/TagYm", tagY);
              Logger.recordOutput("DriveToTag/TagZm", tagZ);
              Logger.recordOutput(
                  "DriveToTag/TagRelativeYawDeg", Units.radiansToDegrees(tagRelativeYaw));
              Logger.recordOutput("DriveToTag/Ambiguity", lastAmbiguity[0]);
              Logger.recordOutput("DriveToTag/TimeSinceLastObsS", timeSinceLastObs);

              // Errors (how far from the goal state)
              double forwardError = tagX - 1.0;
              double lateralError = tagY;
              double headingErrorDeg =
                  Units.radiansToDegrees(MathUtil.angleModulus(tagRelativeYaw - Math.PI));
              double distanceToTag = Math.hypot(tagX, tagY);
              Logger.recordOutput("DriveToTag/ForwardErrorM", forwardError);
              Logger.recordOutput("DriveToTag/LateralErrorM", lateralError);
              Logger.recordOutput("DriveToTag/HeadingErrorDeg", headingErrorDeg);
              Logger.recordOutput("DriveToTag/DistanceToTagM", distanceToTag);

              // PID outputs (raw and final)
              Logger.recordOutput("DriveToTag/RawForward", rawForward);
              Logger.recordOutput("DriveToTag/RawLateral", rawLateral);
              Logger.recordOutput("DriveToTag/RawHeading", rawHeading);
              Logger.recordOutput("DriveToTag/VxMps", vx);
              Logger.recordOutput("DriveToTag/VyMps", vy);
              Logger.recordOutput("DriveToTag/OmegaRadps", omega);

              // At-setpoint flags
              Logger.recordOutput(
                  "DriveToTag/ForwardAtSetpoint", forwardController.atSetpoint());
              Logger.recordOutput(
                  "DriveToTag/LateralAtSetpoint", lateralController.atSetpoint());
              Logger.recordOutput(
                  "DriveToTag/HeadingAtSetpoint", headingController.atSetpoint());
              Logger.recordOutput(
                  "DriveToTag/AllAtSetpoint",
                  forwardController.atSetpoint()
                      && lateralController.atSetpoint()
                      && headingController.atSetpoint());

              // Camera offset (verify in logs that these match your physical setup)
              Logger.recordOutput("DriveToTag/CameraForwardM", VisionConstants.camera0ForwardM);
              Logger.recordOutput("DriveToTag/CameraLeftM", VisionConstants.camera0LeftM);
              Logger.recordOutput("DriveToTag/CameraUpM", VisionConstants.camera0UpM);

              // Robot pose from pose estimator (for context / field visualization)
              Logger.recordOutput("DriveToTag/RobotPose", drive.getPose());
            },
            drive)
        .beforeStarting(
            () -> {
              lastCameraToTag[0] = null;
              lastObservationTimestamp[0] = 0.0;
              lastResultTimestamp[0] = -1.0;
              tagVisible[0] = false;
              forwardController.reset();
              lateralController.reset();
              headingController.reset();
              // Re-apply setpoints after reset (reset clears m_haveSetpoint)
              forwardController.setSetpoint(1.0);
              lateralController.setSetpoint(0.0);
              headingController.setSetpoint(Math.PI);
              xFilter.reset();
              yFilter.reset();
              yawFilter.reset();
              leds.setState(LEDSubsystem.LEDState.TAG_LOST);
              Logger.recordOutput("DriveToTag/CommandStarted", true);
            })
        .until(
            () ->
                tagVisible[0]
                    && forwardController.atSetpoint()
                    && lateralController.atSetpoint()
                    && headingController.atSetpoint())
        .finallyDo(
            (interrupted) -> {
              drive.stop();
              Logger.recordOutput("DriveToTag/CommandStarted", false);
              Logger.recordOutput("DriveToTag/Interrupted", interrupted);
            });
  }

  /**
   * Aim directly at AprilTag 8 and drive to the desired range while held. The robot rotates to
   * center the tag in the camera view and drives forward/backward to reach the target distance.
   * The driver retains strafing control (left stick X). When tag 8 is not visible, the robot holds
   * position.
   */
  @SuppressWarnings("resource")
  public static Command aimAndRangeToTag8(
      Drive drive, DoubleSupplier strafeSupplier) {
    PhotonCamera camera = new PhotonCamera(VisionConstants.camera0Name);

    // Desired state
    final double SPEED_CLAMP = 0.5;
    final double DESIRED_RANGE_M = 1.5; // target distance from tag 8
    final double TURN_KP = 0.01; // per degree of yaw error → fraction of max angular speed
    final double RANGE_KP = 0.15; // per meter of range error → fraction of max linear speed

    // Low-pass filters to smooth noisy vision measurements (reduces motor oscillation)
    LinearFilter yawFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    LinearFilter rangeFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    return Commands.run(
            () -> {
              // Read latest camera result and look for tag 8
              boolean targetVisible = false;
              double targetYaw = 0.0;
              double targetRange = 0.0;

              var result = camera.getLatestResult();
              if (result.hasTargets()) {
                for (var target : result.getTargets()) {
                  if (target.getFiducialId() == 8) {
                    targetYaw = target.getYaw(); // degrees, positive = right
                    targetRange =
                        target.bestCameraToTarget.getTranslation().toTranslation2d().getNorm();
                    targetVisible = true;
                    break;
                  }
                }
              }

              // Strafing from joystick (squared for precision, with deadband)
              double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(), DEADBAND);
              strafe = Math.copySign(strafe * strafe, strafe);

              double forward = 0.0;
              double turn = 0.0;

              if (targetVisible) {
                // Filter noisy vision measurements to prevent motor oscillation
                double filteredYaw = yawFilter.calculate(targetYaw);
                double filteredRange = rangeFilter.calculate(targetRange);

                // Aim: drive yaw toward zero (center tag in camera view)
                turn = -filteredYaw * TURN_KP * drive.getMaxAngularSpeedRadPerSec();
                // Range: drive forward/backward to reach desired distance
                forward =
                    (filteredRange - DESIRED_RANGE_M) * RANGE_KP
                        * drive.getMaxLinearSpeedMetersPerSec();

                // Clamp to 30% max speed for smoother control
                forward =
                    MathUtil.clamp(
                        forward,
                        -drive.getMaxLinearSpeedMetersPerSec() * SPEED_CLAMP,
                        drive.getMaxLinearSpeedMetersPerSec() * SPEED_CLAMP);
                turn =
                    MathUtil.clamp(
                        turn,
                        -drive.getMaxAngularSpeedRadPerSec() * SPEED_CLAMP,
                        drive.getMaxAngularSpeedRadPerSec() * SPEED_CLAMP);
              }

              Logger.recordOutput("AimAndRange/TargetVisible", targetVisible);
              Logger.recordOutput("AimAndRange/TargetYawDeg", targetYaw);
              Logger.recordOutput("AimAndRange/TargetRangeM", targetRange);
              Logger.recordOutput("AimAndRange/RangeErrorM", targetRange - DESIRED_RANGE_M);
              Logger.recordOutput("AimAndRange/ForwardMps", forward);
              Logger.recordOutput("AimAndRange/TurnRadps", turn);

              // Robot-relative drive: vision controls forward + turn, driver controls strafe
              drive.runVelocity(
                  new ChassisSpeeds(
                      forward, strafe * drive.getMaxLinearSpeedMetersPerSec(), turn));
            },
            drive);
  }

  /**
   * Precise profiled alignment to an exact field pose. Uses trapezoidal motion profiles for smooth,
   * fast, jitter-free convergence to the target (±2 cm, ±1°).
   *
   * @param drive The drive subsystem
   * @param blueAlliancePose Target pose in blue-alliance-origin coordinates (auto-flipped for red)
   */
  @SuppressWarnings("resource")
  public static Command alignToPose(Drive drive, Pose2d blueAlliancePose) {
    ProfiledPIDController xController =
        new ProfiledPIDController(6.0, 0, 0.3, new TrapezoidProfile.Constraints(3.0, 5.0));
    ProfiledPIDController yController =
        new ProfiledPIDController(6.0, 0, 0.3, new TrapezoidProfile.Constraints(3.0, 5.0));
    ProfiledPIDController thetaController =
        new ProfiledPIDController(
            7.0, 0, 0.4, new TrapezoidProfile.Constraints(Math.toRadians(540), Math.toRadians(720)));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(0.02); // 2 cm
    yController.setTolerance(0.02); // 2 cm
    thetaController.setTolerance(Math.toRadians(1.0)); // 1°

    Pose2d[] target = {blueAlliancePose};

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              double vx = xController.calculate(current.getX(), target[0].getX());
              double vy = yController.calculate(current.getY(), target[0].getY());
              double omega =
                  thetaController.calculate(
                      current.getRotation().getRadians(),
                      target[0].getRotation().getRadians());
              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              boolean flip =
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
              target[0] =
                  flip ? FlippingUtil.flipFieldPose(blueAlliancePose) : blueAlliancePose;
              Pose2d current = drive.getPose();
              xController.reset(current.getX());
              yController.reset(current.getY());
              thetaController.reset(current.getRotation().getRadians());
            })
        .until(
            () ->
                xController.atSetpoint()
                    && yController.atSetpoint()
                    && thetaController.atSetpoint())
        .withTimeout(2.0)
        .finallyDo(() -> drive.stop());
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = new Rotation2d();
    double gyroDelta = 0.0;
  }
}