// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

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
    public final IntakeSubsystem laser = new IntakeSubsystem();

    private final SendableChooser<Command> autoChooser;

    public final Command pickupCommand = new SequentialCommandGroup(arm.ArmCommand(-2).withTimeout(0.5), new ParallelCommandGroup(arm.ArmCommand(-2), elevator.ElevatorCommand(0.1), claw.ClawCommand(0.4)));
    public final Command scoreLvl4Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(15),
            elevator.ElevatorCommand(40)
            ).withTimeout(0.5), 
        new ParallelCommandGroup(
            arm.ArmCommand(15),
            elevator.ElevatorCommand(38),
            claw.ClawCommand(-0.5)
            )
        );
    public final Command scoreLvl3Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(10),
            elevator.ElevatorCommand(25)
            ).withTimeout(0.5), 
        new ParallelCommandGroup(
            arm.ArmCommand(10),
            elevator.ElevatorCommand(25),
            claw.ClawCommand(-0.5)
            )
        );

    public final Command scoreLvl2Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(13),
            elevator.ElevatorCommand(1)
            ).withTimeout(0.5), 
        new ParallelCommandGroup(
            arm.ArmCommand(11),
            elevator.ElevatorCommand(1),
            claw.ClawCommand(-0.5)
            )
        );

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureBindings() {
        elevator.setDefaultCommand(elevator.ElevatorCommand(2));
        arm.setDefaultCommand(arm.ArmCommand(3));
        claw.setDefaultCommand(claw.ClawCommand(0));
        
        climb.setDefaultCommand(climb.ClimbCommand(0));

        joystick.x().whileTrue(climb.ClimbCommand(0.5));
        joystick.a().whileTrue(climb.ClimbCommand(-0.5));


        operationsController.a().negate()
        .and(operationsController.b().negate())
        .and(operationsController.x().negate())
        .and(operationsController.y().negate())
        .and(operationsController.leftTrigger().negate())
        .and(operationsController.rightTrigger().negate()).whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(2),
                arm.ArmCommand(3)));

        // Lvl 1 scoring position
        operationsController.y().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(30),
                arm.ArmCommand(5)
                )
            );

        // Lvl 2 scoring position
        operationsController.b().and(operationsController.rightTrigger().negate()).whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(2),
                arm.ArmCommand(14)
                )
            );

        // Lvl 3 scoring position
        operationsController.x().and(operationsController.rightTrigger().negate()).whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(25),
                arm.ArmCommand(15)
                )
            );

        // Lvl 4 scoring position
        operationsController.a().and(operationsController.rightTrigger().negate()).whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
                )
            );
        // Reset Arm and Elevator position
    operationsController.leftBumper().whileTrue(
            new ParallelCommandGroup(
                elevator.ElevatorCommand(0),
                arm.ArmCommand(0)
            )
        );

        // Intake
        operationsController.leftTrigger().whileTrue(pickupCommand);
        operationsController
            .povUp()
            .whileTrue(
                new ParallelCommandGroup(
                    elevator.ElevatorCommand(40), 
                    arm.ArmCommand(19)));

        // release command
        operationsController.rightTrigger()
            .and(operationsController.x().negate())
            .and(operationsController.a().negate())
            .and(operationsController.b().negate())
            .whileTrue(claw.ClawCommand(-0.5));

        // Score lvl 4
        operationsController.rightTrigger()
        .and(operationsController.a()).whileTrue(scoreLvl4Command);

        operationsController.rightTrigger()
        .and(operationsController.x()).whileTrue(scoreLvl3Command);

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftTrigger().and(joystick.leftBumper().negate())
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed * 0.5)
                .withVelocityY(joystick.getLeftX() * MaxSpeed * 0.5)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.leftBumper().and(joystick.leftTrigger().negate()).whileTrue(
            drivetrain.applyRequest(
                () ->
                driveRobotCentric.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        joystick.leftBumper().and(joystick.leftTrigger()).whileTrue(
            drivetrain.applyRequest(
                () ->
                driveRobotCentric.withVelocityX(joystick.getLeftY() * MaxSpeed * 0.5)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed * 0.5)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*joystick.b().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));*/

        joystick.b().onTrue(drivetrain.resetGyro());


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return autoChooser.getSelected();
    } catch (Exception e) {
        DriverStation.reportError("No Auto Selected: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    
    }
}
