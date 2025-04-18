package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    RobotConfig config;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private static boolean useMegaTag2 = true; // set to false to use MegaTag1
    private static boolean doRejectUpdate = false;
    private static String limelightUsed;
    private static LimelightHelpers.PoseEstimate LLposeEstimate;
    private static double limelightFrontAvgTagArea = 0;
    private static double limelightBackAvgTagArea = 0;

    public boolean onTargetLL = false;



    private static final Field2d m_field = new Field2d();
    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    

    // Create varibles for autoalign
    double x = 0;
    double y = 0;
    double d = 0; 

    // PID loops for autoalign
    private final PIDController xController = new PIDController(2.3, 0, 0.1);
    private final PIDController yController = new PIDController(2.3, 0, 0.1);
    private final PIDController deltaController = new PIDController(2.3, 0, 0.1);
    

    private double vx;
    private double vy;
    private double vd;

    // Apriltag Positioning request
    private final SwerveRequest.RobotCentric positionRequest = new SwerveRequest.RobotCentric();

    /* Applier for Robot relative speeds for PathPlanner */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();


    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        this.getPigeon2().setYaw(0);
        deltaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
  // Assuming this is a method in your drive subsystem
  
  
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        this.getPigeon2().setYaw(0);
        deltaController.enableContinuousInput(-Math.PI, Math.PI);
  }
    


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
        this.getPigeon2().setYaw(0);
        deltaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) 
    {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command resetGyro()
    {
        return runOnce(
            () -> 
            {
                this.seedFieldCentric();
            }
        );
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) 
    {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotation: ", getState().Pose.getRotation().getDegrees());

        updateOdometry();
        get_manual_LL_Estimate();
        SmartDashboard.putData("Field", m_field);

        Pose2d currentPose = getState().Pose;
        
        m_field.setRobotPose(currentPose); // Fused pose I think
        Double[] fusedPose = {currentPose.getX(), currentPose.getY(), currentPose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Fused PoseDBL", fusedPose);


        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        
         //set to false to use MegaTag1
        /*boolean doRejectUpdate = false;
        if(useMegaTag2 == false)
        {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
            {
                if(mt1.rawFiducials[0].ambiguity > .7)
                {
                    doRejectUpdate = true;
                }
                if(mt1.rawFiducials[0].distToCamera > 3)
                {
                doRejectUpdate = true;
                }
            }
            if(mt1.tagCount == 0)
            {
                doRejectUpdate = true;
            }

      if(!doRejectUpdate)
      {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        this.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        this.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        this.addVisionMeasurement(
            mt2.pose,
            Utils.fpgaToCurrentTime(mt2.timestampSeconds));
            SmartDashboard.putNumber("LL Pose x: ", mt2.pose.getX());
            SmartDashboard.putNumber("LL Pose y: ", mt2.pose.getY());
      }
    } */
    SmartDashboard.putNumber("Robot PoseX:", this.getState().Pose.getX());
    SmartDashboard.putNumber("Robot PoseY:", this.getState().Pose.getY());
    
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) 
        {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        });
        
        // SmartDashboard.putNumber("rtX: ", x);
        // SmartDashboard.putNumber("rtY: ", y);
        // SmartDashboard.putNumber("rtD: ", d);
        
    }
    
    // Printing limelight output to SmartDashboard
    }
    
    public void resetToVision(){
        choose_LL();
        
        LLposeEstimate = get_manual_LL_Estimate();
        if (LLposeEstimate != null) {
            resetPose(LLposeEstimate.pose);
        }
    }

    private void updateOdometry() {
        // feed rotation for mt2
        LimelightHelpers.SetRobotOrientation("limelight", getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation("limelight-top", getState().Pose.getRotation().getDegrees(),
            0, 0, 0, 0, 0);
        choose_LL();

        LLposeEstimate = get_manual_LL_Estimate();
        if (LLposeEstimate != null) {
            this.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            this.addVisionMeasurement(
                LLposeEstimate.pose,
                Utils.fpgaToCurrentTime(LLposeEstimate.timestampSeconds)
            );
            SmartDashboard.putNumber("Timestamp: ", Utils.fpgaToCurrentTime(LLposeEstimate.timestampSeconds));
            SmartDashboard.putNumber("LL Pose x: ", LLposeEstimate.pose.getX());
            SmartDashboard.putNumber("LL Pose y: ", LLposeEstimate.pose.getY());
        }
    }

    private static void choose_LL(){
        // Get limelight botpose 
        limelightFrontAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[11])[10];
        limelightBackAvgTagArea = NetworkTableInstance.getDefault().getTable("limelight-top").getEntry("botpose").getDoubleArray(new double[11])[10];
        
        SmartDashboard.putNumber("Front Limelight Tag Area", limelightFrontAvgTagArea);
        SmartDashboard.putNumber("Top Limelight Tag Area", limelightBackAvgTagArea);    
        
        // Check area of img for each limelight, take largest for visioning
        if(limelightFrontAvgTagArea > 
            limelightBackAvgTagArea){
                limelightUsed = "limelight";
            }
            else
            {
                limelightUsed = "limelight-top";
        }
        
        SmartDashboard.putString("Limelight Used", limelightUsed);
    }

    // private LimelightHelpers.PoseEstimate get_LL_Estimate(boolean useMegaTag2){
    //     doRejectUpdate = false;
    //     LimelightHelpers.PoseEstimate poseEstimate = new LimelightHelpers.PoseEstimate();

    //     if (useMegaTag2 == false) {
    //         poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightUsed);

    //         if (poseEstimate == null){
    //             doRejectUpdate = true;
    //         }
    //         else{
    //             if (poseEstimate.tagCount == 1 && poseEstimate.rawFiducials.length == 1) {
    //                 if (poseEstimate.rawFiducials[0].ambiguity > .7) {
    //                     doRejectUpdate = true;
    //                 }
    //                 if (poseEstimate.rawFiducials[0].distToCamera > 3) {
    //                     doRejectUpdate = true;
    //                 }
    //                 }
    //                 if (poseEstimate.tagCount == 0) {
    //                 doRejectUpdate = true;
    //                 }
    //         }
    //     } else if (useMegaTag2 == true) {
    //         LimelightHelpers.SetRobotOrientation("limelight", getState().Pose.getRotation().getDegrees(),
    //         0, 0, 0, 0, 0);
    //         LimelightHelpers.SetRobotOrientation("limelight-top", getState().Pose.getRotation().getDegrees(),
    //         0, 0, 0, 0, 0);
    //         poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUsed);

    //         if (poseEstimate == null) {
    //             doRejectUpdate = true;
    //         } else {
    //             if (Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second,
    //                                                     // ignore vision updates. Might need to reduce to ~180
    //             {
    //                 doRejectUpdate = true;
    //             }
    //             if (poseEstimate.tagCount == 0) {
    //                 doRejectUpdate = true;
    //             }
    //         }
    //     }

    //     if (doRejectUpdate){
    //         return null;
    //     }
    //     else{
    //         SmartDashboard.putString("LL Pose", poseEstimate.pose.toString());
    //         return poseEstimate;
    //     }
    // }

    private LimelightHelpers.PoseEstimate get_manual_LL_Estimate()
        {
        // Check for targets
        choose_LL();

        // init a pose estimate to hold Limelight
        LimelightHelpers.PoseEstimate poseEstimate = new LimelightHelpers.PoseEstimate();
        double[] botPose = LimelightHelpers.getBotPose(limelightUsed);
        
        // Check for data in botpose
        SmartDashboard.putNumberArray("Botpose", botPose);
        if (botPose.length != 0){
            if (botPose[0] == 0){
                return null;
                // return nothing if data invalid
            }
            // Get pose estimate if data valid
            poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightUsed);
        }

        // Split and display pose estimate
        Double[] pose = {poseEstimate.pose.getX(), poseEstimate.pose.getY(), poseEstimate.pose.getRotation().getRadians()};
        SmartDashboard.putNumberArray("Manual Pose", pose);
        // return pose estimate
        return poseEstimate;
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);

    }
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                     // PID constants for translation
                    new PIDConstants(7.5, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7.5, 0,0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command path_find_to(Pose2d pose, LinearVelocity endVelocity){
        // Drive torward specified field-relative position with given constraints
        return AutoBuilder.pathfindToPose(pose, 
        new PathConstraints(
            2, 
            1,
            // angular speeds and acceleration parameters
            0.5*3.141592, 
            0.25*3.141592), 
            endVelocity);
    }

    public Command positionFromTagCommand(Pose2d targetOffset, String limelight)
    {
        return run(() ->{

        // Get values of target position
        double tx = targetOffset.getX();
        double ty = targetOffset.getY();
        double td = targetOffset.getRotation().getRadians();

        // Retrive limelight data
        Pose3d position = LimelightHelpers.getBotPose3d_TargetSpace(limelight);
        
        // Get values from Limelight
        x = position.getX();
        y = position.getZ();
        d = position.getRotation().getY();

        // Calulate velocities with PID
        vx = xController.calculate(x, tx);
        vy = yController.calculate(y, ty);     
        vd = deltaController.calculate(d, td);
        
        // Push numbers to smartDasboard
        SmartDashboard.putNumber("VX: ", vx);
        SmartDashboard.putNumber("VY: ", vy);
        SmartDashboard.putNumber("VD: ", vd);

        SmartDashboard.putNumber("measured rotation to target: ", d);

        SmartDashboard.putNumber("tagID", LimelightHelpers.getFiducialID(limelight));

        // Check to see if:
        // 1: not seeing Apriltag or disconnected limelight 
        // (as all values from invalid tags are either -1 or null)
        if (!(LimelightHelpers.getFiducialID(limelight) > -1))
        {
            vx = 0;
            vy = 0;
            vd = 0;
        }
        // 2: if on target
        else if (Math.abs(vx) < 0.04 && Math.abs(vy) < 0.04 && Math.abs(vd) < 0.1) {
            onTargetLL = true;
        }
        else
        {
            onTargetLL = false;
        }
        
        // Run swerve with calculated velocities
        this.setControl(
            positionRequest
                .withVelocityX(vy)
                .withVelocityY(-vx)
                .withRotationalRate(-Math.copySign(Math.min(Math.abs(vd), 2), vd))
            );
        }
        )
        // Turn off onTarget flag when finished with autotargeting
        .finallyDo(() -> onTargetLL = false);
            
    }
}

