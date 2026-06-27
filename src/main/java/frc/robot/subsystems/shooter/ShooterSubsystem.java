package frc.robot.subsystems.shooter;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

/**
 * Shooter subsystem — two NEO flywheel motors (leader + inverted follower)
 * and one angle-adjustment motor with software soft limits.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ── CAN IDs ───────────────────────────────────────────────────────────────
    private static final int LEADER_CAN_ID    = 61;
    private static final int FOLLOWER_CAN_ID  = 62;
    private static final int ANGLE_CAN_ID     = 22;
    private static final int ROTATION_CAN_ID  = 48;

    // ── Alliance hub positions (meters) ────────────────────────────────────────
    private static final double BLUE_HUB_X = 4.57;
    private static final double BLUE_HUB_Y = 4.0;
    private static final double RED_HUB_X = 11.97;
    private static final double RED_HUB_Y = 4.0;

    // ── Limits ────────────────────────────────────────────────────────────────
    private static final int SHOOTER_CURRENT_LIMIT = 100;
    private static final int ANGLE_NORMAL_CURRENT_LIMIT = 10;

    // ── Rotation soft limits (encoder rotations, encoder starts at 0) ──────
    private static final double REVERSE_SOFT_LIMIT = -34.214;  // Soft limit forward
    private static final double FORWARD_SOFT_LIMIT = 6.357;    // Soft limit back

    // ── Rotation geometry ──────────────────────────────────────────────
    private static final double ROTATION_GEAR_RATIO = 37.5;         // 1:37.5 gear ratio
    private static final double ROTATION_OFFSET_DEG = 20.0;         // Start position offset (10° to right from robot front)
    private static final double ENC_PER_TURRET_DEG  = ROTATION_GEAR_RATIO / 360.0;
    private static final double DEG_PER_ENC         = 360.0 / ROTATION_GEAR_RATIO;

    // ── Trim control ──────────────────────────────────────────────────────
    private static final double TRIM_DELTA_DEG = 1.2;  // degrees per 20ms for D-pad trim

    // ── Rotation PID ───────────────────────────────────────────────────────
    //TODO: TUNE PID (works fine now)
    private static final double ROTATION_kP = 0.1;
    private static final double ROTATION_kI = 0.0;
    private static final double ROTATION_kD = 0.01;
    private static final double ROTATION_MAX_OUTPUT = 0.7;  // limit turret speed (0–1)

    // ── Angle limits (degrees) ─────────────────────────────────────────────
    private static final double MIN_ANGLE_DEG = 45.0;
    private static final double MAX_ANGLE_DEG = 65.0;

    // ── Angle calibration ───────────────────────────────────────────────
    // Two-phase: drive to upper hardstop, detect stall, then lower hardstop.
    // Measures the actual encoder range between both hardstops.
    private static final double ANGLE_CALIB_POWER          = 0.25; // magnitude used both directions (increased for speed)
    private static final int ANGLE_CALIB_CURRENT_LIMIT  = 2;    // amps — low limit during calibration (increased for faster stall detection)
    private static final double ANGLE_CALIB_CURRENT_THRESH = 3.0;  // amps — stall detection threshold (lowered for faster detection)
    private static final double ANGLE_CALIB_VEL_THRESH     = 20.0; // RPM  — near-zero velocity = stall (lowered threshold)
    private static final double ANGLE_CALIB_MIN_TIME       = 0.2;  // seconds — ignore initial inrush (reduced)
    private static final double ANGLE_CALIB_SETTLE_TIME    = 0.05; // seconds — pause between phases (reduced)

    // ── Motor characteristics ────────────────────────────────────────────────
    private static final double NEO_FREE_SPEED_RPM = 5676.0;

    // ── Flywheel PID (velocity control) ────────────────────────────────────
    private static final double FLYWHEEL_kP = 0.0;
    private static final double FLYWHEEL_kI = 0.0000004;
    private static final double FLYWHEEL_kD = 0.0;
    private static final double FLYWHEEL_kFF = 1.0 / NEO_FREE_SPEED_RPM; // feedforward: 1/maxRPM
    private static final double FLYWHEEL_I_MAX_ACCUM = 0.15; // cap I-term output to prevent windup
    private static final double FLYWHEEL_I_ZONE = 500; // RPM — I-term only active within this error

    // ── Angle PID ─────────────────────────────────────────────────────────────
    private static final double ANGLE_kP = 0.13;
    private static final double ANGLE_kI = 0.0;
    private static final double ANGLE_kD = 0.0;

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final SparkMax leaderMotor;
    private final SparkMax followerMotor;
    private final SparkMax angleMotor;
    private final SparkMax rotationMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;
    private final RelativeEncoder rotationEncoder;
    private final SparkClosedLoopController rotationPID;
    private final SparkClosedLoopController flywheelPID;

    // ── Shooter state ─────────────────────────────────────────────────────────
    private boolean running = false;
    private boolean wasEnabled = false; // tracks enable/disable transitions for rotation motor
    private double lastRPMSetpoint = 0.0;  // track last RPM setpoint for ready check

    // ── Auto-aim turret tracking ────────────────────────────────────────────
    private DoubleSupplier robotHeadingDegSupplier = () -> 0.0;
    private DoubleSupplier autoAimSupplier = null;
    private DoubleSupplier distanceToHubSupplier = () -> 0.0;
    private DoubleSupplier distanceSupplier = () -> 0.0;  // for shot map
    private BooleanSupplier shotMapEnabledSupplier = () -> false;  // enable shot map when true
    private BooleanSupplier rotationAllowedSupplier = () -> true;
    private double aimOffsetDeg = 0.0;
    private double rotationSetpointEnc = 0.0;
    private double targetFieldHeadingDeg = 0.0;  // target heading for logging
    private ShotMap shotMap = null;

    // ── Rewind detection ─────────────────────────────────────────────────────
    // If turret position error exceeds this threshold (encoder rotations),
    // the turret is considered to be rewinding and shooting is blocked.
    // Set to 2.0 rotations (~19.2°) to avoid false triggers during normal tracking
    private static final double REWIND_THRESHOLD_ENC = 2.0;

    // ── Hood angle calibration state ──────────────────────────────────────────
    private enum AngleState {
        WAITING_FOR_ENABLE,
        SEEKING_UPPER,       // driving toward upper hardstop
        SETTLING_UPPER,      // brief pause after upper stall detected
        SEEKING_LOWER,       // driving toward lower hardstop
        CALIBRATED
    }
    private AngleState angleState = AngleState.WAITING_FOR_ENABLE;
    private double angleBottomPos = 0.0;
    private double angleTopPos    = 0.0;
    private double anglePhaseStart = 0.0;
    private double angleUpperRawPos = 0.0;
    private boolean angleCalibrated = false;

    // ── Hood angle tracking ─────────────────────────────────────────────
    private double hoodTargetDeg = MIN_ANGLE_DEG;
    private boolean hoodTracking = false;

    // ── Drive reference for hub calculations ────────────────────────────────
    private Drive drive = null;

    // ─────────────────────────────────────────────────────────────────────────
    public ShooterSubsystem() {

        leaderMotor   = new SparkMax(LEADER_CAN_ID,   MotorType.kBrushless);
        followerMotor = new SparkMax(FOLLOWER_CAN_ID, MotorType.kBrushless);
        angleMotor    = new SparkMax(ANGLE_CAN_ID,    MotorType.kBrushless);
        angleEncoder  = angleMotor.getEncoder();
        anglePID      = angleMotor.getClosedLoopController();

        // ── Leader (with flywheel velocity PID) ─────────────────────────────
        flywheelPID = leaderMotor.getClosedLoopController();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        ClosedLoopConfig flywheelCL = new ClosedLoopConfig();
        flywheelCL.p(FLYWHEEL_kP).i(FLYWHEEL_kI).d(FLYWHEEL_kD);
        flywheelCL.velocityFF(FLYWHEEL_kFF);
        flywheelCL.iMaxAccum(FLYWHEEL_I_MAX_ACCUM);
        flywheelCL.iZone(FLYWHEEL_I_ZONE);
        leaderConfig.apply(flywheelCL);
        leaderMotor.configure(leaderConfig,
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // ── Follower (inverted) ───────────────────────────────────────────────
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.idleMode(IdleMode.kCoast);
        followerConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        followerConfig.follow(LEADER_CAN_ID, true);
        followerMotor.configure(followerConfig,
            ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        // ── Rotation motor (with soft limits + PID) ─────────────────────────────
        rotationMotor   = new SparkMax(ROTATION_CAN_ID, MotorType.kBrushless);
        rotationEncoder = rotationMotor.getEncoder();
        rotationPID     = rotationMotor.getClosedLoopController();

        SparkMaxConfig rotationConfig = new SparkMaxConfig();
        rotationConfig.idleMode(IdleMode.kCoast); // starts in coast; switches to brake on enable
        rotationConfig.smartCurrentLimit(ANGLE_NORMAL_CURRENT_LIMIT);

        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits
            .forwardSoftLimit((float) FORWARD_SOFT_LIMIT)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float) REVERSE_SOFT_LIMIT)
            .reverseSoftLimitEnabled(true);
        rotationConfig.apply(softLimits);

        ClosedLoopConfig rotationCL = new ClosedLoopConfig();
        rotationCL.p(ROTATION_kP).i(ROTATION_kI).d(ROTATION_kD);
        rotationCL.outputRange(-ROTATION_MAX_OUTPUT, ROTATION_MAX_OUTPUT);
        rotationConfig.apply(rotationCL);

        rotationMotor.configure(rotationConfig,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // ── Angle motor (with soft limits) ──────────────────────────────────────
        configureAngleMotor();
    }

    // ── Shooter API ───────────────────────────────────────────────────────────

    public void shoot() { shootAtPower(1.0); }

    /** Sets flywheel to the requested RPM using closed-loop velocity PID. */
    public void shootAtRPM(double rpm) {
        running = Math.abs(rpm) > 50;
        lastRPMSetpoint = rpm;  // Track setpoint for ready check
        flywheelPID.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void shootAtPower(double power) {
        running = Math.abs(power) > 0.05;
        leaderMotor.set(power);
    }

    public void stop() {
        running = false;
        leaderMotor.set(0);
    }

    public boolean isRunning() { return running; }

    // ── Rotation API ──────────────────────────────────────────────────────────

    public void setRotationPower(double power) {
        double pos = rotationEncoder.getPosition();
        if (power > 0 && pos >= FORWARD_SOFT_LIMIT) { rotationMotor.set(0); return; }
        if (power < 0 && pos <= REVERSE_SOFT_LIMIT) { rotationMotor.set(0); return; }
        rotationMotor.set(power);
    }

    public void stopRotation() { rotationMotor.set(0); }

    /** Reset turret encoder position to zero (calibration aid). */
    public void resetRotationEncoder() {
        rotationEncoder.setPosition(0);
        System.out.println("[Shooter] Turret encoder reset to 0");
    }

    /** Enable/disable hood angle tracking (manual D-pad control). */
    public void enableHoodTracking(boolean enabled) {
        hoodTracking = enabled;
    }

    /**
     * Point the turret at the given heading.
     * 0° = robot front, positive = clockwise.
     * Takes the shortest path that stays within soft limits.
     */
    public void setRotationDegrees(double targetDeg) {
        // Normalize target to [0, 360)
        targetDeg = ((targetDeg % 360.0) + 360.0) % 360.0;

        // Base encoder position for this heading
        double rawEnc = (targetDeg - ROTATION_OFFSET_DEG) * ENC_PER_TURRET_DEG;

        // Find the candidate within soft limits closest to current position
        double currentEnc = rotationEncoder.getPosition();
        double bestEnc = Double.NaN;
        double bestDist = Double.MAX_VALUE;

        for (int k = -1; k <= 1; k++) {
            double candidate = rawEnc + k * ROTATION_GEAR_RATIO;
            if (candidate >= REVERSE_SOFT_LIMIT && candidate <= FORWARD_SOFT_LIMIT) {
                double dist = Math.abs(candidate - currentEnc);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestEnc = candidate;
                }
            }
        }

        if (!Double.isNaN(bestEnc)) {
            rotationSetpointEnc = bestEnc;
            rotationPID.setSetpoint(bestEnc, ControlType.kPosition);
        }
    }

    /** Current turret heading in degrees (0 = robot front, CW positive). */
    public double getRotationDegrees() {
        double deg = rotationEncoder.getPosition() * DEG_PER_ENC + ROTATION_OFFSET_DEG;
        return ((deg % 360.0) + 360.0) % 360.0;
    }

    /**
     * Point the turret at a field-relative heading.
     * Converts to robot-relative by subtracting the robot's current heading.
     *
     * @param fieldDeg         desired field-relative heading (0° = downfield, CW positive)
     * @param robotHeadingDeg  robot's current field heading in degrees (e.g. from gyro)
     */
    public void setRotationFieldRelative(double fieldDeg, double robotHeadingDeg) {
        setRotationDegrees(robotHeadingDeg - fieldDeg);
    }

    // ── Trim control (D-pad) ───────────────────────────────────────────────────

    /** Adjust aim offset left (D-pad left). Trim persists on top of auto-aim. */
    public void rotateTrimLeft() {
        adjustAimOffset(TRIM_DELTA_DEG);
    }

    /** Adjust aim offset right (D-pad right). Trim persists on top of auto-aim. */
    public void rotateTrimRight() {
        adjustAimOffset(-TRIM_DELTA_DEG);
    }

    /** Adjust hood angle up (D-pad up). Only works after calibration. */
    public void adjustHoodUp() {
        if (angleCalibrated) adjustHoodTarget(TRIM_DELTA_DEG);
    }

    /** Adjust hood angle down (D-pad down). Only works after calibration. */
    public void adjustHoodDown() {
        if (angleCalibrated) adjustHoodTarget(-TRIM_DELTA_DEG);
    }

    // ── Auto-aim tracking API ───────────────────────────────────────────────

    public void setRobotHeadingSupplier(DoubleSupplier supplier) {
        this.robotHeadingDegSupplier = supplier;
    }

    /** Set the supplier that computes the desired field-relative aim angle (degrees). */
    public void setAutoAimSupplier(DoubleSupplier supplier) {
        this.autoAimSupplier = supplier;
    }

    /** Set the supplier that returns distance to the alliance hub (meters). */
    public void setDistanceToHubSupplier(DoubleSupplier supplier) {
        this.distanceToHubSupplier = supplier;
    }

    /** Alias for setDistanceToHubSupplier - for clarity. */
    public void setDistanceSupplierForDisplay(DoubleSupplier supplier) {
        this.distanceToHubSupplier = supplier;
    }

    /** Set the supplier that gates turret rotation (true = allowed). */
    public void setRotationAllowedSupplier(BooleanSupplier supplier) {
        this.rotationAllowedSupplier = supplier;
    }

    /** Set the supplier for distance to hub (for shot map). */
    public void setDistanceSupplier(DoubleSupplier supplier) {
        this.distanceSupplier = supplier;
    }

    /** Set the shot map for auto RPM and angle. */
    public void setShotMap(ShotMap shotMap) {
        this.shotMap = shotMap;
    }

    /** Set the supplier for enabling shot map (true = apply auto RPM and angle). */
    public void setShotMapEnabledSupplier(BooleanSupplier supplier) {
        this.shotMapEnabledSupplier = supplier;
    }

    /** Set the drive subsystem reference for hub distance calculations. */
    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    /** Adjust the D-pad trim offset on top of auto-aim. */
    public void adjustAimOffset(double deltaDeg) {
        aimOffsetDeg += deltaDeg;
    }

    public double getAimOffsetDeg() {
        return aimOffsetDeg;
    }

    // ── Hood tracking API ────────────────────────────────────────────────────

    /** Adjust the hood target by deltaDeg. Clamped to [MIN_ANGLE_DEG, MAX_ANGLE_DEG]. */
    public void adjustHoodTarget(double deltaDeg) {
        if (!angleCalibrated) return;  // Only allow when calibration is complete
        hoodTargetDeg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, hoodTargetDeg + deltaDeg));
        hoodTracking = true;
    }

    /** Set the hood target to an exact angle. Clamped to [MIN_ANGLE_DEG, MAX_ANGLE_DEG]. */
    public void setHoodTargetDeg(double deg) {
        if (!angleCalibrated) return;  // Only allow when calibration is complete
        hoodTargetDeg = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, deg));
        hoodTracking = true;
    }

    public double getHoodTargetDeg() {
        return hoodTargetDeg;
    }

    /** Total available rotation range in degrees. */
    public static double getTotalRotationRangeDeg() {
        return (FORWARD_SOFT_LIMIT - REVERSE_SOFT_LIMIT) * DEG_PER_ENC;
    }

    // ── Angle API ─────────────────────────────────────────────────────────────

    /**
     * Set angle in degrees. Clamped to [MIN_ANGLE_DEG, MAX_ANGLE_DEG].
     * Only works after calibration. Bottom = MIN_ANGLE_DEG, top = MAX_ANGLE_DEG.
     */
    public void setAngleDegrees(double degrees) {
        if (!angleCalibrated) return;
        degrees = Math.max(MIN_ANGLE_DEG, Math.min(MAX_ANGLE_DEG, degrees));
        double t = (degrees - MIN_ANGLE_DEG) / (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
        double target = angleBottomPos + (angleTopPos - angleBottomPos) * t;
        target = Math.max(angleBottomPos, Math.min(angleTopPos, target));
        anglePID.setSetpoint(target, ControlType.kPosition);
    }

    /** Current angle in degrees. */
    public double getAngleDegrees() {
        double range = angleTopPos - angleBottomPos;
        if (range == 0) return MIN_ANGLE_DEG;
        double t = (angleEncoder.getPosition() - angleBottomPos) / range;
        return MIN_ANGLE_DEG + t * (MAX_ANGLE_DEG - MIN_ANGLE_DEG);
    }

    public boolean isAngleCalibrated() { return angleCalibrated; }

    /** True when the turret is doing a large reposition (rewind past soft limit). */
    public boolean isRewinding() {
        return Math.abs(rotationEncoder.getPosition() - rotationSetpointEnc) > REWIND_THRESHOLD_ENC;
    }

    /** Check if shooter is ready: rotation, angle, and RPM all within tolerance. */
    public boolean isShooterReady() {
        if (isRewinding()) return false;
        if (!angleCalibrated) return false;
        if (!running) return false;

        // Rotation tolerance: 5% of total range (~3.25°)
        double rotationTolerance = getTotalRotationRangeDeg() * 0.05;
        double rotationError = Math.abs(getRotationDegrees() - (targetFieldHeadingDeg - robotHeadingDegSupplier.getAsDouble()));
        if (rotationError > 180) rotationError = 360 - rotationError;  // Handle wraparound
        if (rotationError > rotationTolerance) return false;

        // Angle tolerance: 5% of angle range (~1°)
        double angleTolerance = (MAX_ANGLE_DEG - MIN_ANGLE_DEG) * 0.05;
        double angleError = Math.abs(getAngleDegrees() - hoodTargetDeg);
        if (angleError > angleTolerance) return false;

        // RPM tolerance: 5% of target or 250 RPM, whichever is larger
        double flywheelVelocity = leaderMotor.getEncoder().getVelocity();
        double rpmTolerance = Math.max(Math.abs(lastRPMSetpoint) * 0.05, 250);
        double rpmError = Math.abs(flywheelVelocity - lastRPMSetpoint);
        if (rpmError > rpmTolerance) return false;

        return true;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // ── Rotation motor enable/disable transitions ─────────────────────────
        boolean isEnabled = DriverStation.isEnabled();
        if (isEnabled && !wasEnabled) {
            // Robot just enabled: switch to brake
            aimOffsetDeg = 0.0;
            SparkMaxConfig brakeConfig = new SparkMaxConfig();
            brakeConfig.idleMode(IdleMode.kBrake);
            rotationMotor.configure(brakeConfig,
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Enabled — brake mode ON");
        } else if (!isEnabled && wasEnabled) {
            // Robot just disabled: switch to coast
            SparkMaxConfig coastConfig = new SparkMaxConfig();
            coastConfig.idleMode(IdleMode.kCoast);
            rotationMotor.configure(coastConfig,
                ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
            System.out.println("[Shooter] Disabled — rotation motor coast mode");
        }
        wasEnabled = isEnabled;

        // ── Angle calibration state machine (two-phase hardstop detection) ────
        if (angleState == AngleState.WAITING_FOR_ENABLE && DriverStation.isEnabled()) {
            // Apply low current limit for safe hardstop seeking
            applyAngleCalibCurrentLimit();
            angleState = AngleState.SEEKING_UPPER;
            anglePhaseStart = Timer.getFPGATimestamp();
            angleMotor.set(ANGLE_CALIB_POWER);  // positive = toward upper hardstop
            System.out.println("[Shooter] Angle calibration — seeking upper hardstop");
        }

        if (angleState == AngleState.SEEKING_UPPER) {
            double elapsed = Timer.getFPGATimestamp() - anglePhaseStart;
            if (elapsed > ANGLE_CALIB_MIN_TIME
                    && angleMotor.getOutputCurrent() > ANGLE_CALIB_CURRENT_THRESH
                    && Math.abs(angleEncoder.getVelocity()) < ANGLE_CALIB_VEL_THRESH) {
                angleMotor.set(0);
                angleUpperRawPos = angleEncoder.getPosition();
                angleState = AngleState.SETTLING_UPPER;
                anglePhaseStart = Timer.getFPGATimestamp();
                System.out.println("[Shooter] Upper hardstop detected at encoder " + angleUpperRawPos);
            }
        }

        if (angleState == AngleState.SETTLING_UPPER) {
            if (Timer.getFPGATimestamp() - anglePhaseStart > ANGLE_CALIB_SETTLE_TIME) {
                angleState = AngleState.SEEKING_LOWER;
                anglePhaseStart = Timer.getFPGATimestamp();
                angleMotor.set(-ANGLE_CALIB_POWER);  // negative = toward lower hardstop
                System.out.println("[Shooter] Angle calibration — seeking lower hardstop");
            }
        }

        if (angleState == AngleState.SEEKING_LOWER) {
            double elapsed = Timer.getFPGATimestamp() - anglePhaseStart;
            if (elapsed > ANGLE_CALIB_MIN_TIME
                    && angleMotor.getOutputCurrent() > ANGLE_CALIB_CURRENT_THRESH
                    && Math.abs(angleEncoder.getVelocity()) < ANGLE_CALIB_VEL_THRESH) {
                angleMotor.set(0);
                double lowerRawPos = angleEncoder.getPosition();
                System.out.println("[Shooter] Lower hardstop detected at encoder " + lowerRawPos);

                // Zero encoder at lower hardstop; top is the measured delta
                angleEncoder.setPosition(0);
                angleBottomPos = 0.0;
                angleTopPos = angleUpperRawPos - lowerRawPos;
                angleCalibrated = true;
                angleState = AngleState.CALIBRATED;
                hoodTargetDeg = MIN_ANGLE_DEG;

                // Restore normal current limit and apply soft limits
                configureAngleMotor();
                applyAngleSoftLimits();
                System.out.println("[Shooter] Angle calibrated. Bottom=0, Top=" + angleTopPos
                        + " (range=" + Math.abs(angleUpperRawPos - lowerRawPos) + " enc rotations)");
            }
        }

        // ── Turret field-relative tracking (only when allowed) ─────────────────
        if (rotationAllowedSupplier.getAsBoolean()) {
            double robotHeadingDeg = robotHeadingDegSupplier.getAsDouble();
            if (autoAimSupplier != null) {
                double targetFieldDeg = autoAimSupplier.getAsDouble() + aimOffsetDeg;
                targetFieldHeadingDeg = targetFieldDeg;  // track for logging
                setRotationFieldRelative(targetFieldDeg, robotHeadingDeg);
            } else {
                targetFieldHeadingDeg = aimOffsetDeg;  // track for logging
                setRotationFieldRelative(aimOffsetDeg, robotHeadingDeg);
            }
        } else {
            stopRotation();
        }

        // ── Rewind handling — stop RPM and set angle to minimal ───────────────
        if (isRewinding()) {
            stop();  // Stop flywheel immediately
            setAngleDegrees(MIN_ANGLE_DEG);  // Set hood to minimal angle
            hoodTracking = false;  // Disable hood tracking during rewind
        }
        // ── Auto RPM and angle from shot map (only when LB held and not rewinding) ────────────────
        else if (shotMap != null && shotMapEnabledSupplier.getAsBoolean()) {
            double distance = distanceSupplier.getAsDouble();
            double targetRPM = shotMap.getRPM(distance);
            double targetAngle = shotMap.getAngle(distance);

            // Auto set RPM and angle based on distance
            shootAtRPM(targetRPM);
            setAngleDegrees(targetAngle);
            hoodTracking = false;  // Disable hood tracking while shot map is active (prevents angle jump)

            Logger.recordOutput("Shooter/ShotMapDistance",    distance);
            Logger.recordOutput("Shooter/ShotMapRPM",         targetRPM);
            Logger.recordOutput("Shooter/ShotMapAngle",       targetAngle);
        } else {
            // ── Hood angle tracking (after calibration) ─── only when shot map NOT active
            if (hoodTracking && angleCalibrated) {
                setAngleDegrees(hoodTargetDeg);
            }
        }

        Logger.recordOutput("Shooter/IsRunning",         running);
        Logger.recordOutput("Shooter/LeaderCurrent",     leaderMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/FollowerCurrent",   followerMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/LeaderVelocity",    leaderMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/FollowerVelocity",  followerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Shooter/AngleCurrent",      angleMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/AnglePosition",     angleEncoder.getPosition());
        Logger.recordOutput("Shooter/AngleVelocity",     angleEncoder.getVelocity());
        Logger.recordOutput("Shooter/AngleDegrees",      getAngleDegrees());
        Logger.recordOutput("Shooter/AngleCalibrated",   angleCalibrated);
        Logger.recordOutput("Shooter/RotationPosition",  rotationEncoder.getPosition());
        Logger.recordOutput("Shooter/RotationDegrees",   getRotationDegrees());
        Logger.recordOutput("Shooter/RotationRotation",  Rotation2d.fromDegrees(getRotationDegrees()));
        Logger.recordOutput("Shooter/TargetHeadingDeg",  targetFieldHeadingDeg);
        Logger.recordOutput("Shooter/TargetHeadingRotation", Rotation2d.fromDegrees(targetFieldHeadingDeg));
        Logger.recordOutput("Shooter/AimOffsetDeg",      aimOffsetDeg);
        Logger.recordOutput("Shooter/HoodTargetDeg",     hoodTargetDeg);

        // ── SmartDashboard ────────────────────────────────────────────────────
        SmartDashboard.putNumber("Shooter/Speed (RPM)",      leaderMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Angle (deg)",      getAngleDegrees());
        SmartDashboard.putNumber("Shooter/Distance to Hub (m)", distanceToHubSupplier.getAsDouble());
        SmartDashboard.putNumber("Shooter/Rotation Encoder", rotationEncoder.getPosition());
        SmartDashboard.putNumber("Shooter/Rotation Degrees", getRotationDegrees());
        SmartDashboard.putNumber("Shooter/Angle Encoder",    angleEncoder.getPosition());
    }

    // ── Angle motor configuration ─────────────────────────────────────────────

    private void configureAngleMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(ANGLE_NORMAL_CURRENT_LIMIT);

        // No soft limits before calibration — they are applied after
        ClosedLoopConfig closedLoop = new ClosedLoopConfig();
        closedLoop.p(ANGLE_kP).i(ANGLE_kI).d(ANGLE_kD);
        config.apply(closedLoop);

        angleMotor.configure(config,
            ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Temporarily lower the current limit for safe hardstop seeking. */
    private void applyAngleCalibCurrentLimit() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(ANGLE_CALIB_CURRENT_LIMIT);
        angleMotor.configure(config,
            ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Apply soft limits after calibration finds the real encoder range. */
    private void applyAngleSoftLimits() {
        SparkMaxConfig config = new SparkMaxConfig();
        SoftLimitConfig softLimits = new SoftLimitConfig();
        softLimits
            .forwardSoftLimit((float) angleTopPos)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit((float) angleBottomPos)
            .reverseSoftLimitEnabled(true);
        config.apply(softLimits);
        angleMotor.configure(config,
            ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // ── Hub calculations ───────────────────────────────────────────────────────

    /** Calculate distance from robot to alliance hub. */
    public double calculateDistanceToHub() {
        if (drive == null) return 0.0;

        boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
        double targetX = isBlueAlliance ? BLUE_HUB_X : RED_HUB_X;
        double targetY = isBlueAlliance ? BLUE_HUB_Y : RED_HUB_Y;

        var robotPose = drive.getPose();
        double deltaX = targetX - robotPose.getX();
        double deltaY = targetY - robotPose.getY();
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /** Calculate field-relative angle to alliance hub. */
    public double calculateHubAimAngle() {
        if (drive == null) return 0.0;

        boolean isBlueAlliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
            == DriverStation.Alliance.Blue;
        double targetX = isBlueAlliance ? BLUE_HUB_X : RED_HUB_X;
        double targetY = isBlueAlliance ? BLUE_HUB_Y : RED_HUB_Y;

        var robotPose = drive.getPose();
        double deltaX = targetX - robotPose.getX();
        double deltaY = targetY - robotPose.getY();
        double angleRad = Math.atan2(deltaX, deltaY);
        double angleDeg = Math.toDegrees(angleRad);

        return angleDeg - 90.0;  // Apply 90° offset
    }
}
