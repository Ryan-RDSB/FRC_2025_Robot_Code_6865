// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkMax motor2 = new SparkMax(10, MotorType.kBrushless);

  // Initialize the closed loop controller
SparkClosedLoopController controller2 = motor2.getClosedLoopController();


  /** Creates a new Subsystem. */
  public ArmSubsystem() 
  {
    
    SparkMaxConfig config2 = new SparkMaxConfig();
    config2.closedLoop
    // Set PID gains for position control in slot 0.
    // We don't have to pass a slot number since the default is slot 0.
    .p(0)
    .i(0)
    .d(0)
    .outputRange(0, 5000);

    // Apply configs - reset old parameters, and persist through power-cycles. 
    motor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command ArmCommand(double position_degrees) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          controller2.setReference(position_degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0);

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
