package frc.robot.subsystems.spindexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class SpindexerSubsystem extends SubsystemBase {

    private static final int SPINDEXER_CAN_ID = 59;
    private static final int FEEDER_CAN_ID = 30;
    private static final int SPINDEXER_CURRENT_LIMIT = 120;
    private static final int FEEDER_CURRENT_LIMIT = 150;  // NEO 1.1 peak is ~166A; 150 allows fast spin-up

    private static final double SPINDEXER_POWER = 1.0;
    private static final double FEEDER_POWER = 1.0;

    // NEO 1.1 free speed: 5880 RPM — used only for ready-check threshold
    private static final double FEEDER_MAX_RPM = 5880.0;
    private static final double FEEDER_READY_THRESHOLD = 0.9;  // 90% of max

    private final SparkMax spindexerMotor;
    private final SparkMax feederMotor;

    public SpindexerSubsystem() {
        spindexerMotor = new SparkMax(SPINDEXER_CAN_ID, MotorType.kBrushless);
        feederMotor    = new SparkMax(FEEDER_CAN_ID, MotorType.kBrushless);

        SparkMaxConfig spindexerConfig = new SparkMaxConfig();
        spindexerConfig.idleMode(IdleMode.kBrake);
        spindexerConfig.smartCurrentLimit(SPINDEXER_CURRENT_LIMIT);
        spindexerConfig.inverted(true);
        spindexerMotor.configure(
            spindexerConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters);

        // Feeder uses open-loop percent output — same as REV Hardware Client.
        // kPersistParameters writes to flash to overwrite any stale follower/ramp config.
        // No ramp rate: ramp was limiting output to ~5% due to command scheduler interruptions.
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.idleMode(IdleMode.kCoast);
        feederConfig.smartCurrentLimit(FEEDER_CURRENT_LIMIT);
        feederConfig.inverted(false);
        REVLibError feederConfigResult = feederMotor.configure(
            feederConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
        System.out.println("[Feeder] configure() result: " + feederConfigResult);

        spindexerMotor.set(0);
        feederMotor.set(0);

        setDefaultCommand(Commands.run(this::stop, this));
    }

    /** Runs both spindexer and feeder at full speed. */
    public void runAtFullSpeed() {
        spindexerMotor.set(-SPINDEXER_POWER);
        feederMotor.set(FEEDER_POWER);
    }

    /** Runs only the feeder, spindexer stays off. */
    public void runFeederOnly() {
        spindexerMotor.set(0);
        feederMotor.set(FEEDER_POWER);
    }

    /** Runs only the spindexer, feeder stays off. */
    public void runSpindexerOnly() {
        spindexerMotor.set(-SPINDEXER_POWER);
        feederMotor.set(0);
    }

    public void stop() {
        spindexerMotor.set(0);
        feederMotor.set(0);
    }

    /** Returns the feeder motor velocity in RPM. */
    public double getFeederVelocity() {
        return feederMotor.getEncoder().getVelocity();
    }

    /** Returns true if feeder has reached 90% of max RPM. */
    public boolean isFeederReady() {
        return getFeederVelocity() >= (FEEDER_MAX_RPM * FEEDER_READY_THRESHOLD);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Spindexer/Current", spindexerMotor.getOutputCurrent());
        Logger.recordOutput("Spindexer/Velocity", spindexerMotor.getEncoder().getVelocity());
        Logger.recordOutput("Feeder/Current", feederMotor.getOutputCurrent());
        Logger.recordOutput("Feeder/Velocity", feederMotor.getEncoder().getVelocity());
        Logger.recordOutput("Feeder/AppliedOutput", feederMotor.getAppliedOutput());
    }
}
