// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operationsController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    public final ClawSubsystem claw = new ClawSubsystem();
    public final ClimbSubsystem climb = new ClimbSubsystem();

    public final Command pickupCommand = new SequentialCommandGroup(arm.ArmCommand(0).withTimeout(0.3), new ParallelCommandGroup(arm.ArmCommand(0), elevator.ElevatorCommand(0.5), claw.ClawCommand(0.4)));

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        elevator.setDefaultCommand(elevator.ElevatorCommand(2));
        arm.setDefaultCommand(arm.ArmCommand(3));
        claw.setDefaultCommand(claw.ClawCommand(0));
        
        climb.setDefaultCommand(climb.ClimbCommand(0));

        joystick.x().whileTrue(climb.ClimbCommand(0.5));
        joystick.a().whileTrue(climb.ClimbCommand(-0.5));



        // Lvl 1 scoring
        operationsController.y().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(30),
                arm.ArmCommand(5)
                )
            );

        // Lvl 2 scoring
        operationsController.b().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(20),
                arm.ArmCommand(12)
                )
            );

        // Lvl 3 scoring
        operationsController.x().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(40),
                arm.ArmCommand(14)
                )
            );

        // Lvl 4 scoring
        operationsController.a().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
                )
            );
        // Reset Arm and Elevator
    operationsController.leftBumper().onTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(0),
                arm.ArmCommand(0)
            )
        );

        // Intake
        operationsController.leftTrigger().onTrue(pickupCommand);
        operationsController
            .povUp()
            .whileTrue(
                new ParallelCommandGroup(
                    elevator.ElevatorCommand(40), 
                    arm.ArmCommand(19)));

        operationsController.rightTrigger().whileTrue(claw.ClawCommand(-0.3));
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftBumper().whileTrue(
            drivetrain.applyRequest(() ->
                driveRobotCentric.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    ));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
