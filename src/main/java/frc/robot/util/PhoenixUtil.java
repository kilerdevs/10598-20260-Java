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

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public final class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /**
   * Bridges a TalonFX motor controller to MapleSim physics. Feeds simulated encoder positions and
   * velocities into the TalonFX sim state and reads back the motor voltage command.
   */
  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;
    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }

  /**
   * Extends TalonFXMotorControllerSim to also feed mechanism angle/velocity into a remote
   * CANcoder's sim state, used for steer motors with FusedCANcoder feedback.
   */
  public static class TalonFXMotorControllerWithRemoteCancoderSim
      extends TalonFXMotorControllerSim {
    private final CANcoderSimState remoteCancoderSimState;

    public TalonFXMotorControllerWithRemoteCancoderSim(TalonFX talonFX, CANcoder cancoder) {
      super(talonFX);
      this.remoteCancoderSimState = cancoder.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      remoteCancoderSimState.setRawPosition(mechanismAngle);
      remoteCancoderSimState.setVelocity(mechanismVelocity);
      return super.updateControlSignal(
          mechanismAngle, mechanismVelocity, encoderAngle, encoderVelocity);
    }
  }

  /**
   * Generates evenly-spaced odometry timestamps for the current simulation period, matching
   * MapleSim's sub-tick count.
   */
  public static double[] getSimulationOdometryTimeStamps() {
    final double[] odometryTimeStamps =
        new double[SimulatedArena.getSimulationSubTicksIn1Period()];
    for (int i = 0; i < odometryTimeStamps.length; i++) {
      odometryTimeStamps[i] =
          Timer.getFPGATimestamp()
              - 0.02
              + i * SimulatedArena.getSimulationDt().in(Seconds);
    }
    return odometryTimeStamps;
  }

  /**
   * Adjusts SwerveModuleConstants for simulation compatibility. Zeroes encoder offsets, disables
   * motor inversions, and tunes PID gains for sim. No effect on real robot.
   */
  public static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      regulateModuleConstantForSimulation(
          SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
              moduleConstants) {
    if (RobotBase.isReal()) return moduleConstants;

    return moduleConstants
        .withEncoderOffset(0)
        .withDriveMotorInverted(false)
        .withSteerMotorInverted(false)
        .withEncoderInverted(false)
        .withSteerMotorGains(
            new Slot0Configs()
                .withKP(100)
                .withKI(0)
                .withKD(5.0)
                .withKS(0)
                .withKV(2.33)
                .withKA(0)
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign))
        .withSteerMotorGearRatio(18.75)
        .withDriveFrictionVoltage(Volts.of(0.2))
        .withSteerFrictionVoltage(Volts.of(0.2))
        .withSteerInertia(KilogramSquareMeters.of(0.01));
  }
}
