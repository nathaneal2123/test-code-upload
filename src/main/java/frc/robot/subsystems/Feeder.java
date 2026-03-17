package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {

    private final SparkMax feederMotor;

    private static final double FEED_SPEED = 0.6;   // forward feeding speed
    private static final double REVERSE_SPEED = -0.5; // reverse/unjam speed

    public Feeder() {
        // Initialize the motor using the ID from your Constants
        feederMotor = new SparkMax(Constants.FeederConstants.kFeederMotorId, MotorType.kBrushless);
        
        configureMotor();
    }

    private void configureMotor() {
        // Use the 2026 SparkMaxConfig pattern for reliable parameter setting
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false) // Change to true if the motor feeds game pieces backwards
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(40);

        // Apply the configuration to the motor at once
        feederMotor.configure(config, 
            com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, 
            com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
    }

    /** Feeder forward while command is active */
    public Command forwardCommand() {
        return startEnd(
            () -> feederMotor.set(FEED_SPEED),
            () -> feederMotor.set(0)
        ).withName("Feeder.Forward");
    }

    /** Feeder reverse while command is active (for unjams) */
    public Command reverseCommand() {
        return startEnd(
            () -> feederMotor.set(REVERSE_SPEED),
            () -> feederMotor.set(0)
        ).withName("Feeder.Reverse");
    }

    /** Stops feeder */
    public Command stopCommand() {
        return runOnce(() -> feederMotor.set(0)).withName("Feeder.Stop");
    }
}