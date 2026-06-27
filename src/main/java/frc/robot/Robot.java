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

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.leds.LEDSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Top-level robot class. Extends LoggedRobot (AdvantageKit) instead of TimedRobot so that all
 * subsystem inputs are recorded to the log before any logic runs each loop cycle.
 *
 * <p>All subsystems and button bindings live in {@link RobotContainer}. This class only handles
 * the robot lifecycle (init/periodic for each mode) and AdvantageKit logger setup.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private double robotStartTime = 0.0;  // tracks time since power-on for upload indicator

  /** Runs once at robot power-on. Sets up the AdvantageKit logger and builds RobotContainer. */
  public Robot() {
    robotStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Check for valid swerve config
    var modules =
        new SwerveModuleConstants[] {
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight
        };
    for (var constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
      }
    }

    // Instantiate RobotContainer — all subsystems and button bindings live here
    robotContainer = new RobotContainer();
  }

  /** Runs every 20 ms regardless of mode. Polls all commands and subsystem periodic() methods. */
  @Override
  public void robotPeriodic() {
    // Flash LEDs for first 3 seconds after power-on (code upload indicator)
    double elapsedTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - robotStartTime;
    if (elapsedTime < 3.0) {
      robotContainer.getLEDs().setState(LEDSubsystem.LEDState.UPLOADING);
    }

    // Update shooter configuration from dashboard
    robotContainer.updateShooterConfig();

    CommandScheduler.getInstance().run();
  }

  /** Fires once when the robot enters Disabled mode. Sets LEDs to the disabled pattern. */
  @Override
  public void disabledInit() {
    robotContainer.getLEDs().setState(LEDSubsystem.LEDState.DISABLED);
  }

  @Override
  public void disabledPeriodic() {
    // Keep LED disabled state active during disabled mode
    robotContainer.getLEDs().setState(LEDSubsystem.LEDState.DISABLED);
    // Update the starting pose preview so the driver can see it on the field widget
    robotContainer.updateStartPosePreview();
  }

  /** Fires once when autonomous starts. Fetches and schedules the selected auto routine. */
  @Override
  public void autonomousInit() {
    robotContainer.applyStartingPose();
    robotContainer.getLEDs().setState(LEDSubsystem.LEDState.AUTO);
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  /** Fires once when teleop starts. Cancels any still-running auto command. */
  @Override
  public void teleopInit() {
    robotContainer.applyStartingPose();
    robotContainer.getLEDs().setState(LEDSubsystem.LEDState.IDLE);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  /** Fires once when test mode starts. Cancels all running commands for a clean slate. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  /** Advances the MapleSim physics engine one step and logs simulated field state. */
  @Override
  public void simulationPeriodic() {
    robotContainer.updateSimulation();
  }
}