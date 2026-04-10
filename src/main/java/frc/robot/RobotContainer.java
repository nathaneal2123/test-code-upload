// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final IntakeSubsystem intake = new IntakeSubsystem();

    private final HopperSubsystem hopper = new HopperSubsystem();

    public final ShooterSubsystem shooter = new ShooterSubsystem();

    public final Feeder feeder = new Feeder();

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        configurePathPlanner();
        configureBindings();
        configureAutoChooser();
    }

    private void configurePathPlanner() {
        // Register named commands for PathPlanner event markers and autos
        // Use .asProxy() so named commands don't add their subsystem requirements
        // to the auto's SequentialCommandGroup (which would cause conflicts)
        NamedCommands.registerCommand("deployIntake", intake.deployAndRollCommand().asProxy());
        NamedCommands.registerCommand("stowIntake", intake.stowCommand().withTimeout(1.5).asProxy());
        NamedCommands.registerCommand("spinUpShooter",
            shooter.shootBothCommand(ShooterSubsystem.DEFAULT_SHOOT_RPM).asProxy());
        NamedCommands.registerCommand("shoot",
            hopper.pulsedFeedCommand().alongWith(feeder.forwardCommand()).asProxy());
        NamedCommands.registerCommand("stopAll",
            shooter.stopCommand()
                .alongWith(intake.stowCommand().withTimeout(1.5))
                .alongWith(hopper.stopCommand())
                .alongWith(feeder.stopCommand()).asProxy());
        NamedCommands.registerCommand("shootSequence", shootSequence().asProxy());
        NamedCommands.registerCommand("startRollers", intake.intakeCommand().asProxy());
        NamedCommands.registerCommand("stopRollers", intake.stopRollersCommand().asProxy());

        // Bind EventTrigger for path event markers (fires when robot reaches marker during path)
        new EventTrigger("deployIntake").onTrue(intake.deployAndRollCommand());

        try {
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose,
                drivetrain::resetPose,
                () -> drivetrain.getState().Speeds,
                (speeds, feedforwards) -> drivetrain.setControl(
                    new SwerveRequest.ApplyRobotSpeeds().withSpeeds(speeds)
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),   // translation
                    new PIDConstants(7, 0, 0)     // rotation
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                drivetrain
            );
        } catch (Exception e) {
            System.err.println("Failed to configure PathPlanner AutoBuilder: " + e.getMessage());
        }
    }

    private void configureAutoChooser() {
        try {
            autoChooser.setDefaultOption("New Auto", AutoBuilder.buildAuto("New Auto"));
            autoChooser.addOption("Not Human Shooter Side", AutoBuilder.buildAuto("not human shooter side"));
            autoChooser.addOption("Trench 1", AutoBuilder.buildAuto("Trench1 Auto"));
            autoChooser.addOption("Trench 2", AutoBuilder.buildAuto("Trench2 Auto"));
            autoChooser.addOption("Bump 1", AutoBuilder.buildAuto("Bump1 Auto"));
            autoChooser.addOption("Bump 2", AutoBuilder.buildAuto("Bump2 Auto"));
        } catch (Exception e) {
            System.err.println("Failed to build an auto: " + e.getMessage());
            e.printStackTrace();
        }
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        RobotModeTriggers.teleop()
            .onTrue(intake.rezero());

        // Right Trigger - Pivot down while held
        joystick.rightTrigger().whileTrue(intake.deployAndRollCommand());

        // Eject/Reverse on Right Bumper
        joystick.rightBumper().whileTrue(intake.ejectCommand());

        // POV Up - Pivot up while held
        joystick.povUp().whileTrue(intake.pivotUpCommand());

        // POV Down - Pivot up while held
        joystick.povDown().onTrue(intake.pivotMidCommand());

        // Hold X - Hopper + feeder forward
        joystick.x().whileTrue(
            hopper.pulsedFeedCommand().alongWith(feeder.forwardCommand())
        );
        // POV Left - Spin shooters
        joystick.povLeft().whileTrue(shooter.shootBothCommand(ShooterSubsystem.DEFAULT_SHOOT_RPM));

        // Hold Left Trigger - Shoot sequence
        joystick.leftTrigger().whileTrue(shootSequence());

        // Hold Y - Hopper + feeder reverse together
        joystick.y().whileTrue(
            hopper.reverseCommand().alongWith(feeder.reverseCommand())
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Back/Start - Manual pivot up/down
        joystick.back().whileTrue(intake.pivotUpCommand());
        joystick.start().whileTrue(intake.pivotDownCommand());

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command shootSequence() {
        return Commands.parallel(
            intake.pivotMidCommand(),
            shooter.shootBothCommand(ShooterSubsystem.DEFAULT_SHOOT_RPM),
            Commands.waitSeconds(1.0)
                .andThen(Commands.parallel(
                    hopper.pulsedFeedCommand(),
                    feeder.forwardCommand()
                ))
        )
        .withName("ShootSequence");
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}