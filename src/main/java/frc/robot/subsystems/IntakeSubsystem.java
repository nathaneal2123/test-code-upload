package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class IntakeSubsystem extends SubsystemBase {

  private static final double INTAKE_SPEED = -1;
  private static final double PIVOT_SPEED = 0.8;

  // --- ROLLER: Kraken X60 (TalonFX) ---
  private TalonFX rollerKraken = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);

  private SmartMotorControllerConfig rollerConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withTelemetry("IntakeRollerMotor", TelemetryVerbosity.HIGH)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40));

  private SmartMotorController rollerSMC = new TalonFXWrapper(rollerKraken, DCMotor.getKrakenX60(1), rollerConfig);

  // --- PIVOT: Direct SparkMax control ---
  private SparkMax pivotMotor = new SparkMax(Ports.kIntakePivot, MotorType.kBrushless);

  public IntakeSubsystem() {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.inverted(true)
               .idleMode(IdleMode.kBrake)
               .smartCurrentLimit(20);
    pivotMotor.configure(pivotConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
        com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);
  }

  public Command intakeCommand() {
    return runEnd(() -> rollerSMC.setDutyCycle(INTAKE_SPEED), () -> rollerSMC.setDutyCycle(0))
        .withName("Intake.Run");
  }

  public Command stopRollersCommand() {
    return runOnce(() -> rollerSMC.setDutyCycle(0)).withName("Intake.StopRollers");
  }

  public Command ejectCommand() {
    return runEnd(() -> rollerSMC.setDutyCycle(-INTAKE_SPEED), () -> rollerSMC.setDutyCycle(0))
        .withName("Intake.Eject");
  }

  public Command pivotUpCommand() {
    return runEnd(() -> pivotMotor.set(-PIVOT_SPEED), () -> pivotMotor.set(0))
        .withName("IntakePivot.Up");
  }

  public Command pivotDownCommand() {
    return runEnd(() -> pivotMotor.set(PIVOT_SPEED), () -> pivotMotor.set(0))
        .withName("IntakePivot.Down");
  }

  public Command pivotMidCommand() {
    return run(() -> {
        double current = pivotMotor.getEncoder().getPosition();
        pivotMotor.set(current < 0.270 ? PIVOT_SPEED : -PIVOT_SPEED);
    })
    .until(() -> Math.abs(pivotMotor.getEncoder().getPosition() - 0.270) < 0.02)
    .finallyDo(() -> pivotMotor.set(0))
    .withName("IntakePivot.Mid");
  }

  public Command rezero() {
    return Commands.runOnce(() -> pivotMotor.getEncoder().setPosition(0), this)
        .withName("IntakePivot.Rezero");
  }

  public Command stowCommand() {
    return pivotMidCommand().withName("Intake.Stow");
  }

  public Command deployAndRollCommand() {
    return run(() -> pivotMotor.set(PIVOT_SPEED))
        .withTimeout(2)
        .andThen(
            runEnd(
                () -> {
                    pivotMotor.set(PIVOT_SPEED);
                    rollerSMC.setDutyCycle(INTAKE_SPEED);
                },
                () -> {
                    pivotMotor.set(0);
                    rollerSMC.setDutyCycle(0);
                }
            )
        )
        .withName("Intake.DeployAndRoll");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Pivot Raw Encoder", pivotMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake Roller DutyCycle", rollerKraken.getDutyCycle().getValueAsDouble());
  }
}
