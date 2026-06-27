package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.MAXMotionConfig;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem — a pivot (2x NEO, leader + inverted follower) and a compliant-wheel roller (1x NEO).
 *
 * <p><b>Hybrid positioning:</b>
 * <ul>
 *   <li><b>Upper:</b> Encoder-based with MAXMotion PID — gentle return, no slamming.
 *   <li><b>Lower:</b> Current-based — drives down until a current spike indicates the lower hardstop.
 * </ul>
 *
 * <p><b>Calibration:</b> On first enable the pivot drives slowly upward until a current spike
 * indicates the upper hardstop. The encoder is zeroed there. Lower position is discovered on
 * the first downward toggle via current spike detection.
 *
 * <p><b>Roller:</b> Runs when the intake is deployed (LOWER or MOVING_TO_LOWER).
 */
public class IntakeSubsystem extends SubsystemBase {

    // ── CAN IDs ───────────────────────────────────────────────────────────────
    private static final int PIVOT_LEADER_CAN_ID   = 33;
    private static final int PIVOT_FOLLOWER_CAN_ID = 45;
    private static final int ROLLER_CAN_ID         = 35;

    // Intake mechanism is incomplete — all motor output is blocked until this is false.
    private static final boolean INTAKE_DISABLED = false;

    // ── Drive power for going DOWN (current-based) ───────────────────────────
    private static final double DRIVE_DOWN_POWER = -0.3; // negative = toward lower hardstop

    // ── Roller power ──────────────────────────────────────────────────────────
    private static final double ROLLER_POWER = -5800.0 / 5880.0;

    // ── Calibration (slow upward drive to find upper hardstop) ───────────────
    private static final double CALIB_POWER              = 0.12;  // slow, gentle
    private static final double CALIB_CURRENT_THRESHOLD  = 5.0;    // amps
    private static final double CALIB_VELOCITY_THRESHOLD = 50.0;   // RPM (near-zero)
    private static final double CALIB_MIN_MOVE_TIME      = 0.3;    // seconds before checking
    private static final double CALIB_TIMEOUT_SEC        = 3.0;    // failsafe

    // ── Current-spike detection for lower hardstop ───────────────────────────
    private static final double STALL_CURRENT_THRESHOLD  = 15.0;   // amps
    private static final double STALL_VELOCITY_THRESHOLD = 50.0;   // RPM (near-zero)
    private static final double STALL_MIN_MOVE_TIME      = 0.3;    // seconds before checking
    private static final double STALL_TIMEOUT_SEC        = 5.0;    // safety timeout

    // ── MAXMotion PID (for going UP — gentle encoder-based return) ───────────
    private static final double kP          = 0.13;
    private static final double kI          = 0.0;
    private static final double kD          = 0.0;
    private static final double kMaxOutput  =  0.80;
    private static final double kMinOutput  = -0.80;

    // ── Holding PID (when idle at UPPER — eco mode to keep straight) ─────────
    private static final double HOLD_kP     = 0.05;  // low power to be eco
    private static final double HOLD_kI     = 0.0;
    private static final double HOLD_kD     = 0.0;
    private static final double HOLD_kMaxOutput = 0.15;  // very low power output
    private static final double HOLD_kMinOutput = -0.15;
    private static final double CRUISE_VEL  = 2000.0; // RPM
    private static final double MAX_ACCEL   = 1500.0;  // RPM/s
    private static final double ALLOWED_ERR = 0.1;    // encoder rotations
    private static final double AT_TARGET_THRESHOLD  = 0.15; // encoder rotations
    private static final double UPPER_MOVE_TIMEOUT   = 2.5;  // seconds

    // ── Safety ────────────────────────────────────────────────────────────────
    private static final int PIVOT_CURRENT_LIMIT  = 40;
    private static final int ROLLER_CURRENT_LIMIT = 30;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final SparkMax                  pivotLeader;
    private final SparkMax                  pivotFollower;
    private final RelativeEncoder           pivotEncoder;
    private final RelativeEncoder           pivotFollowerEncoder;
    private final SparkClosedLoopController pivotPID;
    private final SparkMax                  rollerMotor;

    // ── State machine ─────────────────────────────────────────────────────────
    private enum State {
        WAITING_FOR_ENABLE,
        CALIBRATING,        // slow drive up to find upper hardstop
        UPPER,
        LOWER,
        MOVING_TO_UPPER,    // encoder-based MAXMotion PID (gentle)
        MOVING_TO_LOWER     // current-based (drive until spike)
    }

    private State   state         = State.WAITING_FOR_ENABLE;
    private boolean calibrated    = false;
    private double  positionUpper = 0.0;
    private double  positionLower = Double.NaN; // discovered on first downward toggle
    private double  moveStart     = 0.0;
    private boolean rollersSupressed = false;
    private boolean forcedLower      = false;
    private double  lowerReachedTimestamp = 0.0;
    private boolean holdingAtUpper   = false;  // track if we're using holding PID at UPPER

    // ─────────────────────────────────────────────────────────────────────────
    public IntakeSubsystem() {

        // ── Pivot leader ─────────────────────────────────────────────────────
        pivotLeader  = new SparkMax(PIVOT_LEADER_CAN_ID, MotorType.kBrushless);
        pivotEncoder = pivotLeader.getEncoder();
        pivotPID     = pivotLeader.getClosedLoopController();

        MAXMotionConfig maxMotion = new MAXMotionConfig();
        maxMotion.cruiseVelocity(CRUISE_VEL, ClosedLoopSlot.kSlot0);
        maxMotion.maxAcceleration(MAX_ACCEL, ClosedLoopSlot.kSlot0);
        maxMotion.allowedProfileError(ALLOWED_ERR, ClosedLoopSlot.kSlot0);
        maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal, ClosedLoopSlot.kSlot0);

        ClosedLoopConfig closedLoop = new ClosedLoopConfig();
        closedLoop.p(kP).i(kI).d(kD).outputRange(kMinOutput, kMaxOutput);
        closedLoop.apply(maxMotion);

        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kBrake);
        leaderConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
        leaderConfig.apply(closedLoop);


        pivotLeader.configure(
            leaderConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        pivotEncoder.setPosition(0);

        // ── Pivot follower (inverted) ────────────────────────────────────────
        pivotFollower = new SparkMax(PIVOT_FOLLOWER_CAN_ID, MotorType.kBrushless);
        pivotFollowerEncoder = pivotFollower.getEncoder();

        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kBrake);
        followerConfig.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
        followerConfig.follow(PIVOT_LEADER_CAN_ID, true); // inverted

        pivotFollower.configure(
            followerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // Zero follower encoder on startup
        pivotFollowerEncoder.setPosition(0);

        // ── Compliant-wheel roller ───────────────────────────────────────────
        rollerMotor = new SparkMax(ROLLER_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig rollerConfig = new SparkMaxConfig();
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);

        rollerMotor.configure(
            rollerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /** Toggles the intake between upper and lower positions. Bound to L1. */
    public void toggle() {
        if (!calibrated) {
            DriverStation.reportWarning("[Intake] Toggle ignored — not calibrated yet (state=" + state + ")", false);
            return;
        }
        if (forcedLower) return;

        switch (state) {
            case UPPER:
            case MOVING_TO_UPPER:
                driveDown();
                break;
            case LOWER:
            case MOVING_TO_LOWER:
                moveToUpper();
                break;
            default:
                break;
        }
    }

    /** Lowers the intake for shooting mode (rollers stay off). */
    public void lowerForShooting() {
        rollersSupressed = true;
        if (!calibrated) return;
        if (state == State.UPPER || state == State.MOVING_TO_UPPER) {
            driveDown();
        }
    }

    /** Raises the intake after shooting mode and re-enables rollers. */
    public void raiseFromShooting() {
        rollersSupressed = false;
        if (!calibrated || forcedLower) return;
        if (state == State.LOWER || state == State.MOVING_TO_LOWER) {
            moveToUpper();
        }
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    /** Go UP — encoder-based MAXMotion PID (gentle, no slamming). */
    public void moveToUpper() {
        holdingAtUpper = false;  // reset flag to re-enable holding PID when we reach UPPER
        pivotPID.setSetpoint(positionUpper, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        state     = State.MOVING_TO_UPPER;
        moveStart = Timer.getFPGATimestamp();
        System.out.println("[Intake] Moving to UPPER via PID -> " + positionUpper);
    }

    /** Go DOWN — raw power, stop on current spike (lower hardstop). */
    public void driveDown() {
        pivotLeader.set(DRIVE_DOWN_POWER);
        state     = State.MOVING_TO_LOWER;
        moveStart = Timer.getFPGATimestamp();
        System.out.println("[Intake] Driving to LOWER (current-based)");
    }

    private boolean isStalledLower() {
        return pivotLeader.getOutputCurrent() > STALL_CURRENT_THRESHOLD
            && Math.abs(pivotEncoder.getVelocity()) < STALL_VELOCITY_THRESHOLD;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {

        if (INTAKE_DISABLED) {
            pivotLeader.set(0);
            pivotFollower.set(0);
            rollerMotor.set(0);
            return;
        }

        double currentPos = pivotEncoder.getPosition();

        // ── Force-lower: read dashboard bool, drive down and stay there ──────
        forcedLower = SmartDashboard.getBoolean("Intake/ForceLower", false);
        if (forcedLower && calibrated
                && state != State.LOWER && state != State.MOVING_TO_LOWER) {
            driveDown();
        }

        // ── Start calibration on first enable ─────────────────────────────────
        if (state == State.WAITING_FOR_ENABLE && DriverStation.isEnabled()) {
            state     = State.CALIBRATING;
            moveStart = Timer.getFPGATimestamp();
            pivotLeader.set(CALIB_POWER);
            System.out.println("[Intake] Calibration started — driving slowly to upper hardstop");
        }

        double elapsed = Timer.getFPGATimestamp() - moveStart;

        // ── State machine ────────────────────────────────────────────────────
        switch (state) {

            case CALIBRATING: {
                if (elapsed > CALIB_TIMEOUT_SEC) {
                    pivotLeader.set(0);
                    pivotEncoder.setPosition(0);
                    positionUpper = 0.0;
                    calibrated    = true;
                    state = State.UPPER;
                    DriverStation.reportWarning("[Intake] Calibration timeout — using current position as upper", false);
                } else if (elapsed > CALIB_MIN_MOVE_TIME
                        && pivotLeader.getOutputCurrent() > CALIB_CURRENT_THRESHOLD
                        && Math.abs(pivotEncoder.getVelocity()) < CALIB_VELOCITY_THRESHOLD) {
                    pivotLeader.set(0);
                    pivotEncoder.setPosition(0);
                    positionUpper = 0.0;
                    calibrated    = true;
                    state = State.UPPER;
                    System.out.println("[Intake] Calibrated — encoder zeroed at upper hardstop");
                }
                break;
            }

            case MOVING_TO_UPPER: {
                double error = Math.abs(currentPos - positionUpper);
                if (error < AT_TARGET_THRESHOLD) {
                    state = State.UPPER;
                    System.out.println("[Intake] Reached UPPER");
                } else if (elapsed > UPPER_MOVE_TIMEOUT) {
                    pivotLeader.set(0);
                    state = State.UPPER;
                    System.out.println("[Intake] UPPER movement timeout at pos=" + currentPos);
                }
                break;
            }

            case MOVING_TO_LOWER: {
                if (elapsed > STALL_TIMEOUT_SEC) {
                    pivotLeader.set(0);
                    positionLower = currentPos;
                    state = State.LOWER;
                    lowerReachedTimestamp = Timer.getFPGATimestamp();
                    System.out.println("[Intake] LOWER movement timeout — using pos=" + currentPos);
                } else if (elapsed > STALL_MIN_MOVE_TIME && isStalledLower()) {
                    pivotLeader.set(0);
                    positionLower = currentPos;
                    state = State.LOWER;
                    lowerReachedTimestamp = Timer.getFPGATimestamp();
                    System.out.println("[Intake] Reached LOWER (current spike) at pos=" + currentPos);
                }
                break;
            }

            case UPPER: {
                // Enable holding PID on first entry to UPPER state
                if (!holdingAtUpper) {
                    holdingAtUpper = true;
                    // Switch leader to holding PID (eco mode, low power)
                    ClosedLoopConfig holdingCL = new ClosedLoopConfig();
                    holdingCL.p(HOLD_kP).i(HOLD_kI).d(HOLD_kD).outputRange(HOLD_kMinOutput, HOLD_kMaxOutput);
                    SparkMaxConfig leaderHoldConfig = new SparkMaxConfig();
                    leaderHoldConfig.apply(holdingCL);
                    pivotLeader.configure(leaderHoldConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    pivotPID.setSetpoint(positionUpper, ControlType.kPosition, ClosedLoopSlot.kSlot0);
                    System.out.println("[Intake] Holding at UPPER with eco PID");
                }
                // Keep follower encoder synced with leader
                if (Math.abs(pivotFollowerEncoder.getPosition() - pivotEncoder.getPosition()) > 0.5) {
                    pivotFollowerEncoder.setPosition(pivotEncoder.getPosition());
                }
                break;
            }

            case LOWER:
            case WAITING_FOR_ENABLE:
                holdingAtUpper = false;  // reset flag when leaving UPPER
                break;
        }

        // ── Roller — runs when intake is deployed ────────────────────────────
        boolean rollerShouldRun = calibrated
            && !rollersSupressed
            && !forcedLower
            && (state == State.LOWER || state == State.MOVING_TO_LOWER);
        rollerMotor.set(rollerShouldRun ? ROLLER_POWER : 0.0);

        // ── Logging ──────────────────────────────────────────────────────────
        Logger.recordOutput("Intake/State", state.toString());
        Logger.recordOutput("Intake/Calibrated", calibrated);
        Logger.recordOutput("Intake/EncoderPosition", currentPos);
        Logger.recordOutput("Intake/FollowerEncoderPosition", pivotFollowerEncoder.getPosition());
        Logger.recordOutput("Intake/EncoderSync", Math.abs(pivotFollowerEncoder.getPosition() - currentPos));
        Logger.recordOutput("Intake/PositionUpper", positionUpper);
        Logger.recordOutput("Intake/PositionLower", positionLower);
        Logger.recordOutput("Intake/RollerActive", rollerShouldRun);
        Logger.recordOutput("Intake/PivotLeaderCurrent", pivotLeader.getOutputCurrent());
        Logger.recordOutput("Intake/PivotFollowerCurrent", pivotFollower.getOutputCurrent());
        Logger.recordOutput("Intake/RollerCurrent", rollerMotor.getOutputCurrent());
        Logger.recordOutput("Intake/PivotVelocity", pivotEncoder.getVelocity());
    }

    // ── Status helpers ───────────────────────────────────────────────────────
    public boolean isCalibrated() { return calibrated; }
    public boolean isAtUpper()    { return state == State.UPPER; }
    public boolean isAtLower()    { return state == State.LOWER; }
    /** True when the intake has been at LOWER for at least the given seconds. */
    public boolean isAtLowerFor(double seconds) {
        return state == State.LOWER
            && (Timer.getFPGATimestamp() - lowerReachedTimestamp) >= seconds;
    }
    public boolean isMoving()     { return state == State.MOVING_TO_UPPER || state == State.MOVING_TO_LOWER; }
}
