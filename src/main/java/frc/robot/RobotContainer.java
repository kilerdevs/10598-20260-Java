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

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.ModuleIOTalonFXSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShotMap;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

import frc.robot.subsystems.gamepad.GamepadSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.zones.ZoneActionSubsystem;


import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Central wiring class for the robot. Instantiates every subsystem, creates the controller, and
 * maps all buttons to commands.
 *
 * <p><b>Logitech F310 button map (port 0):</b>
 * <pre>
 *   Left  stick        — Translation (field-relative)
 *   Right stick X      — Rotation
 *   A                  — Hold: snap heading to 0° while driving
 *   B                  — Tap:  reset gyro heading to 0°
 *   X                  — Toggle vertical reverse
 *   Y                  — Toggle horizontal reverse
 *   LB                 — Toggle: deploy / collapse intake
 *   RB                 — Hold: run spindexer (feeder first, then spindexer when ready)
 * </pre>
 */
public class RobotContainer {
  // ── Shooter trim control rate ──────────────────────────────────────────────
  private static final double MANUAL_TRIM_RATE = 0.6;  // degrees per 20ms = 30°/sec

  private double horizontal_reverse = 1.0;
  private double vertical_reverse = 1.0;
  private boolean lbHeld = false;  // LB button state for shot map
  private boolean manualHoodMode = false;  // manual hood angle mode (via D-pad, max RPM when shooting)
  private boolean showOffPrevious = false;  // track ShowOff mode transitions for smooth angle interpolation

  // ── Intake-turret safety interlock ────────────────────────────────────
  // true  = R1 lowers intake, turret only tracks when intake is down + R1 held
  // false = R1 does NOT move intake, turret always tracks the hub
  private boolean intakeTurretSafety = true;

  // ── Swerve drivetrain heading for turret ──────────────────────────────
  // true  = turret auto-aim uses swerve drivetrain heading
  // false = turret assumes robot is not rotating
  private boolean useDrivetrainHeading = true;

  // ── Steering sensitivity ──────────────────────────────────────────────
  // false = normal speed, true = ultra-slow precision mode (10% rotation speed)
  private boolean slowModeEnabled = false;

  // ── Orchestra (Hava Nagila on all Krakens) ──────────────────────────────
  private final Orchestra orchestra = new Orchestra();

  // ── Subsystems ────────────────────────────────────────────────────────────
  private final Drive drive;
  private final Vision vision;
  private final LEDSubsystem leds = new LEDSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final SpindexerSubsystem spindexer = new SpindexerSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ShotMap shotMap = new ShotMap();

  private ZoneActionSubsystem zoneAction; // initialized after drive

  // MapleSim drive simulation — only used in SIM mode, null on real robot
  private SwerveDriveSimulation driveSimulation = null;

  // ── Controller ────────────────────────────────────────────────────────────
  // Logitech F310 on USB port 0 (set in Driver Station)
  private final CommandXboxController controller = new CommandXboxController(0);
  private final GamepadSubsystem gamepad = new GamepadSubsystem(controller);

  // ── Dashboard ─────────────────────────────────────────────────────────────
  private final LoggedDashboardChooser<Command> autoChooser;

  // ── Starting pose selector (editable on dashboard) ─────────────────────
  private final Field2d startPoseField = new Field2d();

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Robot front rotated 90° right: old right side = new front
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontRight),   // new FL = old FR
                new ModuleIOTalonFX(TunerConstants.BackRight),    // new FR = old BR
                new ModuleIOTalonFX(TunerConstants.FrontLeft),    // new BL = old FL
                new ModuleIOTalonFX(TunerConstants.BackLeft),     // new BR = old BL
                (pose) -> {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        break;

      case SIM:
        // Create MapleSim physics simulation
        driveSimulation =
            new SwerveDriveSimulation(
                Drive.mapleSimConfig, new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        // In simulation, modules stay at their default positions — no physical rotation
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontLeft, driveSimulation.getModules()[0]),
                new ModuleIOTalonFXSim(
                    TunerConstants.FrontRight, driveSimulation.getModules()[1]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackLeft, driveSimulation.getModules()[2]),
                new ModuleIOTalonFXSim(
                    TunerConstants.BackRight, driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    camera0Name,
                    robotToCamera0,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    camera1Name,
                    robotToCamera1,
                    driveSimulation::getSimulatedDriveTrainPose));
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (pose) -> {});
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // ── Shooter setup ────────────────────────────────────────────────────────────
    // Wire drive reference and setup auto-aim/shot map
    shooter.setDrive(drive);
    shooter.setRobotHeadingSupplier(() -> useDrivetrainHeading ? drive.getRotation().getDegrees() : 0.0);
    shooter.setAutoAimSupplier(shooter::calculateHubAimAngle);
    shooter.setDistanceSupplier(shooter::calculateDistanceToHub);
    shooter.setDistanceToHubSupplier(shooter::calculateDistanceToHub);  // Display in SmartDashboard
    shooter.setShotMap(shotMap);
    shooter.setShotMapEnabledSupplier(() -> lbHeld);

    // Dashboard control to toggle drivetrain heading
    SmartDashboard.putBoolean("Shooter/UseDrivetrainHeading", useDrivetrainHeading);

    // ── Zone-based action triggers ────────────────────────────────────────
    zoneAction = new ZoneActionSubsystem(drive::getPose);

    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Register each PathPlanner auto — reset odometry to path start before running
    for (String autoName : AutoBuilder.getAllAutoNames()) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      Pose2d startPose = auto.getStartingPose();
      autoChooser.addOption(
          autoName,
          Commands.runOnce(() -> drive.setPose(startPose), drive).andThen(auto));
    }

    // Direct path follow test — bypasses PathPlannerAuto entirely
    try {
      PathPlannerPath testPath = PathPlannerPath.fromPathFile("wokol field");
      autoChooser.addOption(
          "Direct: wokol field",
          Commands.runOnce(
                  () -> testPath.getStartingHolonomicPose().ifPresent(drive::setPose), drive)
              .andThen(AutoBuilder.followPath(testPath)));
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path 'wokol field': " + e.getMessage(), e.getStackTrace());
    }

    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive to AprilTag 1", DriveCommands.driveToAprilTag1(drive, leds));

    // ── Intake-turret safety toggle ─────────────────────────────────────
    SmartDashboard.putBoolean("Turret/IntakeSafety", true);
    SmartDashboard.putBoolean("Intake/ForceLower", false);

    // ── Drive slow mode (affects both movement and rotation) ────────────────
    SmartDashboard.putBoolean("Drive/SlowMode", false);

    // ── Turret encoder reset ──────────────────────────────────────────────────
    SmartDashboard.putBoolean("Turret/ResetEncoder", false);
    new Trigger(() -> SmartDashboard.getBoolean("Turret/ResetEncoder", false))
        .onTrue(Commands.runOnce(() -> {
            shooter.resetRotationEncoder();
            SmartDashboard.putBoolean("Turret/ResetEncoder", false);  // auto-reset the button
        }, shooter).ignoringDisable(true));

    // ── Manual hood angle mode (D-pad up/down control, max RPM when shooting) ──
    SmartDashboard.putBoolean("Shooter/ShowOff", false);

    // ── Starting pose dashboard controls ──────────────────────────────────
    SmartDashboard.putNumber("StartPose/X (m)", 1.5);
    SmartDashboard.putNumber("StartPose/Y (m)", 5.5);
    SmartDashboard.putNumber("StartPose/Rotation (deg)", 0.0);
    SmartDashboard.putData("StartPose/Preview", startPoseField);
    updateStartPosePreview();

    // ── Orchestra (Hava Nagila on all Krakens) ──────────────────────────────
    String canBus = TunerConstants.kCANBus.getName();
    int[] krakenIds = {2, 3, 5, 6, 8, 9, 11, 12};
    for (int id : krakenIds) {
      orchestra.addInstrument(new TalonFX(id, canBus));
    }
    orchestra.loadMusic("nagila_hava.chrp");

    SmartDashboard.putBoolean("Play Hava Nagila", false);
    new Trigger(() -> SmartDashboard.getBoolean("Play Hava Nagila", false))
        .onTrue(Commands.runOnce(() -> orchestra.play()).ignoringDisable(true))
        .onFalse(Commands.runOnce(() -> orchestra.stop()).ignoringDisable(true));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // ── Default drive command ─────────────────────────────────────────────────
    // Left stick = translate, right stick X = rotate (field-relative, alliance-flipped)
    // When slow mode is enabled (via dashboard), both movement and rotation are reduced to 10%
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() * vertical_reverse * (slowModeEnabled ? 0.4 : 1.0),
            () -> -controller.getLeftX() * horizontal_reverse * (slowModeEnabled ? 0.4 : 1.0),
            () -> controller.getRightX() * (slowModeEnabled ? 0.4 : 1.0)));

    // A — hold to lock heading at 0° while still translating freely
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    // Y — toggle horizontal reverse
    controller
    .y()
    .onTrue(
        Commands.runOnce(() -> horizontal_reverse = horizontal_reverse*-1).ignoringDisable(true)


    );
    // X — toggle vertical reverse
    controller
        .x()
        .onTrue(
            Commands.runOnce(() -> vertical_reverse = vertical_reverse * -1)
                .ignoringDisable(true));


    // B — instantly zero the gyro heading (useful after robot is repositioned)
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // □ Square — hold to aim at AprilTag 8 and drive to desired range
    // Vision controls rotation (aim) and forward/backward (range).
    // Driver retains strafing control via left stick X.
    // controller
    //     .square()
    //     .whileTrue(
    //         DriveCommands.aimAndRangeToTag8(drive, () -> -controller.getLeftX()));

    // ── Intake ────────────────────────────────────────────────────────────────

    // LT (60% threshold) — deploy intake and keep it active
    new Trigger(() -> controller.getLeftTriggerAxis() > 0.6)
        .onTrue(new InstantCommand(() -> intake.driveDown(), intake))
        .onFalse(Commands.runOnce(() -> intake.moveToUpper(), intake));

    // D-Pad Left/Right — turret rotation trim (angle offset persists on auto-aim)
    controller
        .povLeft()
        .whileTrue(Commands.run(() -> shooter.rotateTrimLeft(), shooter));
    controller
        .povRight()
        .whileTrue(Commands.run(() -> shooter.rotateTrimRight(), shooter));

    // D-Pad Up/Down — hood angle trim
    controller
        .povUp()
        .whileTrue(Commands.run(() -> shooter.adjustHoodUp(), shooter));
    controller
        .povDown()
        .whileTrue(Commands.run(() -> shooter.adjustHoodDown(), shooter));

    // ── Spindexer / Shooting ──────────────────────────────────────────────────

    // RT — hold to run spindexer and shooter, show ready/not-ready status
    new Trigger(() -> controller.getRightTriggerAxis() > 0.6)
        .whileTrue(Commands.parallel(
            Commands.runOnce(() -> lbHeld = true),
            Commands.run(() -> spindexer.runAtFullSpeed(), spindexer),
            Commands.run(() -> {
                // In manual hood mode: max RPM, angle controlled by D-pad
                // Otherwise: use shot map for RPM and angle
                if (manualHoodMode) {
                    shooter.shootAtRPM(5676.0);  // max RPM (NEO free speed)
                } else {
                    // Auto-aim and set RPM from shot map
                    double distance = shooter.calculateDistanceToHub();
                    double targetRPM = shotMap.getRPM(distance);
                    double targetAngle = shotMap.getAngle(distance);
                    shooter.shootAtRPM(targetRPM);
                    shooter.setAngleDegrees(targetAngle);
                }

                // LED feedback
                if (shooter.isShooterReady()) {
                    leds.setState(LEDSubsystem.LEDState.READY);
                } else {
                    leds.setState(LEDSubsystem.LEDState.NOT_READY);
                }
            }, shooter, leds)
        ))
        .onFalse(Commands.runOnce(() -> {
            lbHeld = false;
            shooter.stop();
            spindexer.stop();
            shooter.setHoodTargetDeg(45.0);
            leds.setState(LEDSubsystem.LEDState.IDLE);
        }, shooter, spindexer, leds));
  }

  public LEDSubsystem getLEDs() {
    return leds;
  }

  public GamepadSubsystem getGamepad() {
    return gamepad;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Advances the MapleSim physics simulation and logs field state. Called from Robot. */
  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }

  /** Resets the simulation field for autonomous. */
  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  // ── Starting pose ────────────────────────────────────────────────────────

  /** Reads X/Y/Rotation from the dashboard and returns the selected starting pose. */
  public Pose2d getStartingPose() {
    double x   = SmartDashboard.getNumber("StartPose/X (m)", 1.5);
    double y   = SmartDashboard.getNumber("StartPose/Y (m)", 5.5);
    double rot = SmartDashboard.getNumber("StartPose/Rotation (deg)", 0.0);
    return new Pose2d(x, y, Rotation2d.fromDegrees(rot));
  }

  /** Applies the dashboard starting pose to the drive odometry. Call on enable. */
  public void applyStartingPose() {
    Pose2d pose = getStartingPose();
    drive.setPose(pose);
    System.out.println("[StartPose] Applied: " + pose);
  }

  /** Updates the Field2d preview widget. Call from disabledPeriodic. */
  public void updateStartPosePreview() {
    startPoseField.setRobotPose(getStartingPose());
  }

  /** Update shooter configuration from dashboard. Call from robotPeriodic. */
  public void updateShooterConfig() {
    useDrivetrainHeading = SmartDashboard.getBoolean("Shooter/UseDrivetrainHeading", true);
    slowModeEnabled = SmartDashboard.getBoolean("Drive/SlowMode", false);
    manualHoodMode = SmartDashboard.getBoolean("Shooter/ShowOff", false);

    // When ShowOff is disabled, reset hood tracking to allow smooth shot map interpolation
    if (showOffPrevious && !manualHoodMode) {
      shooter.enableHoodTracking(false);  // disable manual tracking, allow shot map to take over
    }
    showOffPrevious = manualHoodMode;
  }

  /** Calculate distance from robot to alliance hub. */
}