package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    public static final double DEFAULT_SHOOT_RPM = 3000;

    public Command shootBothCommand(double rpm) { return Commands.none(); }
    public Command spinUpAndWaitCommand(double rpm) { return Commands.none(); }
    public Command shootLeftCommand(double rpm) { return Commands.none(); }
    public Command shootRightCommand(double rpm) { return Commands.none(); }
    public Command stopCommand() { return Commands.none(); }
    public boolean atSpeed(double rpm) { return false; }
}