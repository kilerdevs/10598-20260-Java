package frc.robot.subsystems.leds;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import frc.robot.generated.TunerConstants;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * LED subsystem — drives a CANdle (CAN ID 14) connected to a 49-LED GRB 5V strip.
 *
 * <p>The CANdle has 8 onboard LEDs (indices 0–7) and the external GRB strip occupies indices 8–56.
 * Each {@link LEDState} maps to a distinct animation pair (onboard + strip) so the drive team
 * can read robot state at a glance from across the field.
 *
 * <p>Call {@link #setState(LEDState)} from any subsystem or command. The state only re-applies
 * the animation when it actually changes, avoiding redundant CAN writes.
 */
public class LEDSubsystem extends SubsystemBase {

  /**
   * Robot operating states, each with a unique LED pattern:
   *
   * <ul>
   *   <li>{@link #IDLE}           — purple onboard + rainbow strip (ready, waiting for input)
   *   <li>{@link #INTAKING}       — orange strobe onboard + orange color-flow strip
   *   <li>{@link #SHOOTING}       — fast red strobe everywhere
   *   <li>{@link #AUTO}           — blue breathing onboard + fast rainbow strip
   *   <li>{@link #DISABLED}       — dim orange breathing onboard + fire strip
   *   <li>{@link #ERROR}          — slow red strobe everywhere (distinct from SHOOTING)
   *   <li>{@link #HAS_GAME_PIECE} — solid green everywhere
   *   <li>{@link #ALIGNING}       — solid yellow onboard + yellow larson scanner strip
   *   <li>{@link #TAG_TRACKING}   — solid green everywhere (tag visible and tracking)
   *   <li>{@link #TAG_LOST}       — fast red strobe onboard + solid red strip (tag not visible)
   *   <li>{@link #UPLOADING}      — white fast strobe everywhere (code uploading/syncing)
   * </ul>
   */
  public enum LEDState {
    IDLE,
    INTAKING,
    SHOOTING,
    AUTO,
    DISABLED,
    ERROR,
    HAS_GAME_PIECE,
    ALIGNING,
    TAG_TRACKING,
    TAG_LOST,
    UPLOADING,
    READY,       // Shooter ready to shoot (green strobe)
    NOT_READY    // Shooter not ready (red strobe)
  }

  private static final int CAN_ID = 14;

  // LED index ranges — 0-7 onboard, 8-56 external GRB 5V strip (49 LEDs)
  private static final int ONBOARD_START = 0;
  private static final int ONBOARD_END = 7;
  private static final int STRIP_START = 8;
  private static final int STRIP_END = 56;

  // Animation slots: 0 = onboard, 1 = strip
  private static final int SLOT_ONBOARD = 0;
  private static final int SLOT_STRIP = 1;

  // Colors
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
  private static final RGBWColor RED = new RGBWColor(255, 0, 0);
  private static final RGBWColor ORANGE = new RGBWColor(255, 80, 0);
  private static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
  private static final RGBWColor YELLOW = new RGBWColor(255, 255, 0);
  private static final RGBWColor PURPLE = new RGBWColor(128, 0, 255);

  private final CANdle candle;
  private LEDState currentState = null;

  public LEDSubsystem() {
    candle = new CANdle(CAN_ID, TunerConstants.kCANBus);

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    candle.getConfigurator().apply(config);

    setState(LEDState.DISABLED);
  }

  /**
   * Switch to a new LED state. No-op if the robot is already in that state, which avoids
   * flooding the CAN bus with repeated animation commands.
   */
  public void setState(LEDState state) {
    if (state != currentState) {
      currentState = state;
      applyState(state);
    }
  }

  /** Returns the currently active LED state. */
  public LEDState getState() {
    return currentState;
  }

  private void applyState(LEDState state) {
    switch (state) {
      case IDLE:
        // Everything: purple Larson scanner (default teleop animation)
        candle.setControl(
            new LarsonAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(PURPLE)
                .withFrameRate(15)
                .withSize(3));
        candle.setControl(
            new LarsonAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(PURPLE)
                .withFrameRate(15)
                .withSize(5));
        break;

      case INTAKING:
        // Everything: orange strobe
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(ORANGE)
                .withFrameRate(8));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(ORANGE)
                .withFrameRate(8));
        break;

      case SHOOTING:
        // Onboard: fast red strobe
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(RED)
                .withFrameRate(15));
        // Strip: fast red strobe
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(RED)
                .withFrameRate(15));
        break;

      case AUTO:
        // Everything: blue breathing
        candle.setControl(
            new SingleFadeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(BLUE)
                .withFrameRate(80));
        candle.setControl(
            new SingleFadeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(BLUE)
                .withFrameRate(80));
        break;

      case DISABLED:
        // Both onboard and strip: orange strobe
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(ORANGE)
                .withFrameRate(3));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(ORANGE)
                .withFrameRate(3));
        break;

      case ERROR:
        // Everything: slow red strobe (distinct from SHOOTING)
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(RED)
                .withFrameRate(4));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(RED)
                .withFrameRate(4));
        break;

      case HAS_GAME_PIECE:
        // Everything: solid green
        candle.setControl(
            new SolidColor(ONBOARD_START, ONBOARD_END).withColor(GREEN));
        candle.setControl(
            new SolidColor(STRIP_START, STRIP_END).withColor(GREEN));
        break;

      case ALIGNING:
        // Everything: solid yellow
        candle.setControl(
            new SolidColor(ONBOARD_START, ONBOARD_END).withColor(YELLOW));
        candle.setControl(
            new SolidColor(STRIP_START, STRIP_END).withColor(YELLOW));
        break;

      case TAG_TRACKING:
        // Everything: solid green (tag visible and actively tracking)
        candle.setControl(
            new SolidColor(ONBOARD_START, ONBOARD_END).withColor(GREEN));
        candle.setControl(
            new SolidColor(STRIP_START, STRIP_END).withColor(GREEN));
        break;

      case TAG_LOST:
        // Everything: fast red strobe
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(RED)
                .withFrameRate(10));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(RED)
                .withFrameRate(10));
        break;

      case UPLOADING:
        // Everything: white fast strobe (code upload/sync indicator)
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(new RGBWColor(255, 255, 255))  // white
                .withFrameRate(20));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(new RGBWColor(255, 255, 255))  // white
                .withFrameRate(20));
        break;

      case READY:
        // Everything: fast green strobe (shooter at speed/angle)
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(GREEN)
                .withFrameRate(20));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(GREEN)
                .withFrameRate(20));
        break;

      case NOT_READY:
        // Everything: fast red strobe (shooter not yet at speed/angle)
        candle.setControl(
            new StrobeAnimation(ONBOARD_START, ONBOARD_END)
                .withSlot(SLOT_ONBOARD)
                .withColor(RED)
                .withFrameRate(20));
        candle.setControl(
            new StrobeAnimation(STRIP_START, STRIP_END)
                .withSlot(SLOT_STRIP)
                .withColor(RED)
                .withFrameRate(20));
        break;
    }
  }

  @Override
  public void periodic() {
    Logger.recordOutput("LEDs/State", currentState.toString());
  }
}
