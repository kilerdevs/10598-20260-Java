package frc.robot.subsystems.gamepad;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.littletonrobotics.junction.Logger;

/**
 * Manages Xbox-style controller haptic feedback (rumble / vibration).
 *
 * <p>All rumble methods require an intensity (0.0–1.0) and a duration in seconds.
 * The left and right rumble motors are tracked independently — each has its own expiry timestamp.
 * {@link #periodic()} checks the FPGA timer every 20 ms and zeroes out any expired motor.
 *
 * <p>Call {@link #stopRumble()} to cut all rumble immediately (e.g. on robot disable).
 *
 * <p>Example — short "got game piece" buzz:
 * <pre>
 *   gamepad.rumble(0.8, 0.25);     // both motors, 80%, 250 ms
 * </pre>
 */
public class GamepadSubsystem extends SubsystemBase {

  private final GenericHID controller;

  private double leftRumble = 0.0;
  private double rightRumble = 0.0;
  private double leftExpiry = 0.0;
  private double rightExpiry = 0.0;

  public GamepadSubsystem(CommandXboxController controller) {
    this.controller = controller.getHID();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    if (now >= leftExpiry) leftRumble = 0.0;
    if (now >= rightExpiry) rightRumble = 0.0;

    controller.setRumble(RumbleType.kLeftRumble, leftRumble);
    controller.setRumble(RumbleType.kRightRumble, rightRumble);
    Logger.recordOutput("Gamepad/LeftRumble", leftRumble);
    Logger.recordOutput("Gamepad/RightRumble", rightRumble);
  }

  /** Rumble both motors at the given intensity (0.0–1.0) for {@code seconds}. */
  public void rumble(double intensity, double seconds) {
    double expiry = Timer.getFPGATimestamp() + seconds;
    leftRumble = intensity;
    rightRumble = intensity;
    leftExpiry = expiry;
    rightExpiry = expiry;
  }

  /** Rumble only the left motor (0.0–1.0) for {@code seconds}. */
  public void rumbleLeft(double intensity, double seconds) {
    leftRumble = intensity;
    leftExpiry = Timer.getFPGATimestamp() + seconds;
  }

  /** Rumble only the right motor (0.0–1.0) for {@code seconds}. */
  public void rumbleRight(double intensity, double seconds) {
    rightRumble = intensity;
    rightExpiry = Timer.getFPGATimestamp() + seconds;
  }

  /** Stop all rumble immediately. */
  public void stopRumble() {
    leftRumble = 0.0;
    rightRumble = 0.0;
    leftExpiry = 0.0;
    rightExpiry = 0.0;
  }
}
