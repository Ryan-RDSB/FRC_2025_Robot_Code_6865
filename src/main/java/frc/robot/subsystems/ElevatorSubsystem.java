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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkMax motor0 = new SparkMax(10, MotorType.kBrushless);
SparkMax motor1 = new SparkMax(11, MotorType.kBrushless);
  // Initialize the closed loop controller
SparkClosedLoopController controller0 = motor0.getClosedLoopController();

  /** Creates a new Subsystem. */
  public ElevatorSubsystem()
  {
  
    motor0.getEncoder().setPosition(0);
    motor0.getEncoder().setPosition(0);
    SparkMaxConfig config0 = new SparkMaxConfig();
    config0.closedLoop
    // Set PID gains for position control in slot 0.
    // We don't have to pass a slot number since the default is slot 0.
    .p(10)
    .i(0)
    .d(0)
    .outputRange(-1, 1);
    config0.closedLoop.maxMotion
    .maxVelocity(1800)
    .maxAcceleration(6000)
    .allowedClosedLoopError(1);

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
  public Command ElevatorCommand(double position_meters) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          controller0.setReference(position_meters*30, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        });
  }

  public void toPos(){
    

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
    System.out.println(motor0.getEncoder().getPosition());
    System.out.println(motor1.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
