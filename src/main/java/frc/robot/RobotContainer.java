// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.sql.Driver;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
// import frc.robot.subsystems.OrchestraSubsystem;

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

    public final LEDSubsystem led = new LEDSubsystem();


    private final SendableChooser<Command> autoChooser;

    public final Command pickupCommand = new SequentialCommandGroup(
        arm.ArmCommand(-4).withTimeout(0.5), 
        new ParallelCommandGroup(arm.ArmCommand(-4), 
            elevator.ElevatorCommand(0.1), 
            claw.ClawCommand(0.4)
            )
        );

    public final Command autoPickupCommand = new SequentialCommandGroup(
        arm.ArmCommand(-4).withTimeout(0.5), 
        new ParallelCommandGroup(arm.ArmCommand(-4), 
            elevator.ElevatorCommand(0.1), 
            claw.ClawCommand(0.4)
            )
        );

    // Upright positions
    public final Command holdLvl4Command = new ParallelCommandGroup(
        elevator.ElevatorCommand(40),
        arm.ArmCommand(18)
        );

    public final Command holdLvl3Command = new ParallelCommandGroup(
        elevator.ElevatorCommand(12),
        arm.ArmCommand(16)
        );

    public final Command holdLvl2Command = new ParallelCommandGroup(
        elevator.ElevatorCommand(3),
        arm.ArmCommand(13)
        );

    public final Command holdLvl1Command = new ParallelCommandGroup(
        elevator.ElevatorCommand(27),
        arm.ArmCommand(6)
        );
    
    // Scoring motions for upright
    public final Command scoreLvl4Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(15),
            elevator.ElevatorCommand(40)
            ).withTimeout(0.3), 
        new ParallelCommandGroup(
            arm.ArmCommand(15),
            elevator.ElevatorCommand(38),
            claw.ClawCommand(-0.5)
            )
        );

    public final Command scoreLvl3Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(9),
            elevator.ElevatorCommand(25)
            ).withTimeout(0.3), 
        new ParallelCommandGroup(
            arm.ArmCommand(9),
            elevator.ElevatorCommand(25),
            claw.ClawCommand(-0.5)
            )
        );

    public final Command scoreLvl2Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            arm.ArmCommand(7),
            elevator.ElevatorCommand(1)
            ).withTimeout(0.3), 
        new ParallelCommandGroup(
            arm.ArmCommand(6),
            elevator.ElevatorCommand(1),
            claw.ClawCommand(-0.5)
            )
        );

    public final Command scoreLvl1Command = new SequentialCommandGroup(
        new ParallelCommandGroup(
            elevator.ElevatorCommand(27),
            arm.ArmCommand(6)
            ).withTimeout(0.3),
        new ParallelCommandGroup(
            elevator.ElevatorCommand(27),
            arm.ArmCommand(6),
            claw.ClawCommand(-0.2)
            )
        );


    // integrated scoring commands for auto       
    public final Command autoAlignScoreLvl4Left = new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new ParallelCommandGroup
            (
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
            )
            .until(() -> drivetrain.onTargetLL),
            
            new ParallelCommandGroup
            (
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
            ).withTimeout(0.5),

            new SequentialCommandGroup(
                new ParallelCommandGroup(
                    arm.ArmCommand(15),
                    elevator.ElevatorCommand(40)
                )
                .withTimeout(0.3), 
            new ParallelCommandGroup(
                arm.ArmCommand(15),
                elevator.ElevatorCommand(38),
                claw.ClawCommand(-0.5)
                )
                .withTimeout(1.5)
            )
        ),

        drivetrain.positionFromTagCommand(
                new Pose2d(
                    -0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            )
        );



    public final Command autoAlignScoreLvl4Right = new ParallelDeadlineGroup(

        // While doin this in this order:
        new SequentialCommandGroup(
            // Hold arm up until on target
            new ParallelCommandGroup
            (
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
            )
            .until(() -> drivetrain.onTargetLL),

            new ParallelCommandGroup
            (
                elevator.ElevatorCommand(40),
                arm.ArmCommand(18)
            ).withTimeout(0.5),

            // Bring arm down, then scores
            new SequentialCommandGroup(

                new ParallelCommandGroup(
                    arm.ArmCommand(15),
                    elevator.ElevatorCommand(40)
                )
                .withTimeout(0.3), 
            new ParallelCommandGroup(
                arm.ArmCommand(15),
                elevator.ElevatorCommand(38),
                claw.ClawCommand(-0.5)
                )
                .withTimeout(1.5)
            )
        ),

        // at the same time, move the robot to scoring position
        drivetrain.positionFromTagCommand(
                new Pose2d(
                    0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            )
        );



    public final Command climbDown = climb.ClimbCommand(1);
    public final Command climbUp = climb.ClimbCommand(-1).withTimeout(0.5);

    public RobotContainer() {
        NamedCommands.registerCommand("lvl4Hold", holdLvl4Command);
        NamedCommands.registerCommand("lvl3Hold", holdLvl3Command);
        NamedCommands.registerCommand("lvl2Hold", holdLvl2Command);
        NamedCommands.registerCommand("lvl1Hold", holdLvl1Command);

        NamedCommands.registerCommand("scoreLvl4", scoreLvl4Command);
        NamedCommands.registerCommand("scoreLvl3", scoreLvl3Command);
        NamedCommands.registerCommand("scoreLvl2", scoreLvl2Command);
        NamedCommands.registerCommand("scoreLvl1", scoreLvl1Command);

        NamedCommands.registerCommand("alignRight", 
        drivetrain.positionFromTagCommand(
                new Pose2d(
                    0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            ));

        NamedCommands.registerCommand("alignLeft",
        drivetrain.positionFromTagCommand(
                new Pose2d(
                    -0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            )
        );

        NamedCommands.registerCommand("pickup", pickupCommand);
        NamedCommands.registerCommand("neutralArmAndElevator", 
            new ParallelCommandGroup(
                elevator.ElevatorCommand(5),
                new SequentialCommandGroup(
                    arm.ArmCommand(-2).withTimeout(1),
                    arm.ArmCommand(0)
                )
            )
        );
        NamedCommands.registerCommand("stopClaw", claw.ClawCommand(0));

        NamedCommands.registerCommand("autoAlignScoreLvl4Right", autoAlignScoreLvl4Right);
        NamedCommands.registerCommand("autoAlignScoreLvl4Left", autoAlignScoreLvl4Left);


        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        CameraServer.startAutomaticCapture(0);
        drivetrain.seedFieldCentric();
    }

    private void configureBindings() {
        // Default attachment Behevior
        elevator.setDefaultCommand(elevator.ElevatorCommand(6));
        arm.setDefaultCommand(
            new SequentialCommandGroup(
                arm.ArmCommand(-2).withTimeout(1),
                arm.ArmCommand(0)
                ));
        claw.setDefaultCommand(claw.ClawCommand(0));
        
        climb.setDefaultCommand(climb.ClimbCommand(0));

        led.setDefaultCommand(led.LEDCommand("red"));

        joystick.x().whileTrue(climbDown);
        joystick.y().whileTrue(climbUp);

        joystick.x().and(joystick.rightTrigger()).whileTrue(climb.ClimbCommand(0.5));
        joystick.y().and(joystick.rightTrigger()).whileTrue(climb.ClimbCommand(-0.5));

        // Lvl 1 scoring position
        operationsController.y().whileTrue(
            holdLvl1Command
            );

        // Lvl 2 scoring position
        operationsController.b().and(operationsController.rightTrigger().negate()).whileTrue(
            holdLvl2Command
            );

        // Lvl 3 scoring position
        operationsController.x().and(operationsController.rightTrigger().negate()).whileTrue(
            holdLvl3Command
            );

        // Lvl 4 scoring position
        operationsController.a().and(operationsController.rightTrigger().negate()).whileTrue(
            holdLvl4Command
            );


        // Auto score on target
        operationsController.a().and(() -> drivetrain.onTargetLL).whileTrue(scoreLvl4Command);
        operationsController.x().and(() -> drivetrain.onTargetLL).whileTrue(scoreLvl3Command);
        operationsController.b().and(() -> drivetrain.onTargetLL).whileTrue(scoreLvl2Command);
        // Reset Arm and Elevator position
        // operationsController.leftBumper().whileTrue(
        //     new ParallelCommandGroup(
        //         elevator.ElevatorCommand(0),
        //         arm.ArmCommand(0)
        //     )
        // );

        // release command
        // operationsController.rightTrigger()
        //     .and(operationsController.y().negate())
        //     .and(operationsController.x().negate())
        //     .and(operationsController.a().negate())
        //     .and(operationsController.b().negate())
        //     .whileTrue(claw.ClawCommand(-0.5));

        // Score lvl 4
        operationsController.rightTrigger()
        .and(operationsController.a())
        // .and(operationsController.povDown().negate())
        .whileTrue(scoreLvl4Command);
        // Score lvl 3
        operationsController.rightTrigger()
        .and(operationsController.x()).whileTrue(scoreLvl3Command);
        // Score lvl 2
        operationsController.rightTrigger()
        .and(operationsController.b()).whileTrue(scoreLvl2Command);
        // Score lvl 1
        operationsController.rightTrigger()
        .and(operationsController.y()).whileTrue(scoreLvl1Command);



        /* FIELD CENTRIC DRIVE COMMANDS */
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Field relative half speed
        joystick.leftTrigger().and(joystick.rightTrigger().negate())
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.5)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            )
        );


        /* ROBOT CENTRIC DRIVE COMMANDS */
        // robot centric
        joystick.rightTrigger().and(joystick.leftTrigger().negate()).whileTrue(
            drivetrain.applyRequest(
                () ->
                driveRobotCentric.withVelocityX(joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


        // low speed robot centric
        joystick.leftTrigger().and(joystick.rightTrigger()).whileTrue(
            drivetrain.applyRequest(
                () ->
                driveRobotCentric.withVelocityX(joystick.getLeftY() * MaxSpeed * 0.5)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.5)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * 0.5) // Drive counterclockwise with negative X (left)
            )
        );

        // Drive utils
        joystick.a()
        .and(joystick.rightTrigger().negate())
        .and(joystick.leftTrigger().negate())
        .whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.start().onTrue(drivetrain.resetGyro());

        /* PATHPLANNER-BASED POSITIONING AND SCORING */

        /* HUMAN PLAYER PICKUP POSITIONS */
        // To left-side pickup Position
        joystick.povLeft().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(1, 7, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(-55))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));
        
        // To right-side pickup Position
        joystick.povRight().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(1, 1, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(55))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));

        /* INDIVIDUAL REEF POSITIONS */
        // DS side reef
        joystick.povDown().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(2.91, 4.03, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(0))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));
        
        // DS side left corner reef
        joystick.povDownLeft().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(3.696, 5.362, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(-60))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));

        // Far side left corner reef
        joystick.povUpLeft().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(5.254, 5.362, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(-120))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));
        
        // Far side reef 
        joystick.povUp().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(6.06, 4.03, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(180))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));

        // Far side right corner reef
        joystick.povUpRight().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(5.254, 2.710, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(120))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));

        // Ds side right corner reef
        joystick.povDownRight().whileTrue(
            drivetrain.path_find_to(
                new Pose2d(3.696, 2.710, 
                new Rotation2d(
                    edu.wpi.first.math.util.Units.degreesToRadians(60))
                    ),
                LinearVelocity.ofBaseUnits(0, MetersPerSecond)));

        /* AUTONOMUS SCORING POSITION COMMANDS */
        
        joystick.rightBumper().whileTrue(
            drivetrain.positionFromTagCommand(
                new Pose2d(
                    0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            )
        );

        joystick.leftBumper().whileTrue(
            drivetrain.positionFromTagCommand(
                new Pose2d(
                    -0.13, 
                    -0.66, 
                    new Rotation2d(edu.wpi.first.math.util.Units.degreesToRadians(0))
                ), 
                "limelight-score"
            )
        );
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        //joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        // Autopickup
        // new Trigger(() -> laser.coralIn).whileTrue(new ParallelCommandGroup(autoPickupCommand, led.LEDCommand("green")));
        operationsController.leftBumper().whileTrue(
            pickupCommand
        );
        
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        return autoChooser.getSelected();
    } catch (Exception e) {
        DriverStation.reportError("No Auto Selected: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    
    }
}
