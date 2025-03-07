// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkMax motor0 = new SparkMax(10, MotorType.kBrushless);
SparkMax motor1 = new SparkMax(11, MotorType.kBrushless);
ProfiledPIDController elevatorPidController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0.3,0.3));
ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.00001, 0);

  // Initialize the closed loop controller
SparkClosedLoopController controller0 = motor0.getClosedLoopController();

  /** Creates a new Subsystem. */
  public ElevatorSubsystem()
  {
    
    motor0.getEncoder().setPosition(0);
    motor1.getEncoder().setPosition(0);
    SparkMaxConfig config0 = new SparkMaxConfig();
    config0
    .inverted(true)
    /* .closedLoop
    // Set PID gains for position control in slot 0.
    // We don't have to pass a slot number since the default is slot 0.
    .p(0)
    .i(0)
    .d(0)
    .outputRange(-0.5, 1)
    .positionWrappingEnabled(false);
    config0.closedLoop.maxMotion
    .maxVelocity(4800)
    .maxAcceleration(2400)
    .allowedClosedLoopError(0)*/;

    // Create follower controller
    // Ensures to invert it
    SparkMaxConfig config1 = new SparkMaxConfig();
    config1.follow(10, true);
    // Apply configs - reset old parameters, and persist through power-cycles. 
    motor0.configure(config0, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command ElevatorCommand(double position_rotations) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          toPos(position_rotations);
        });
  }

  public void ElevatorPickup()
  {
    toPos(1);
  }

  public void ElevatorReady()
  {
    toPos(4);
  }

  public void ElevatorMid()
  {
    toPos(39);
  }

  public void ElevatorMax()
  {
    toPos(40);
  }

  public void toPos(double rotations)
  {
    //controller0.setReference(rotations, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, feedforward.calculate(controller0.));
    elevatorPidController.setGoal(rotations);
    motor0.setVoltage(
      elevatorPidController.calculate(
        motor0.getEncoder().getPosition()) 
        + feedforward.calculate(elevatorPidController.getSetpoint().velocity)
        );
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean ElevatorCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
