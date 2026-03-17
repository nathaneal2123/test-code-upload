package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    public Command forwardCommand() { return Commands.none(); }
    public Command reverseCommand() { return Commands.none(); }
    public Command stopCommand() { return Commands.none(); }
}