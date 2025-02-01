// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.io.ObjectInputFilter.Config;

import javax.xml.crypto.dsig.keyinfo.KeyValue;
import com.revrobotics.spark.ClosedLoopSlot;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkFlex m_motor = new SparkFlex(0, MotorType.kBrushless);
  // Initialize the closed loop controller
SparkClosedLoopController m_controller = m_motor.getClosedLoopController();

  /** Creates a new Subsystem. */
  public ElevatorSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command ElevatorCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          m_controller.setReference(0, ControlType.kPosition);
          SparkFlexConfig config = new SparkFlexConfig();
        });
  }

  public void toPos(){
    m_controller.setReference(0, ControlType.kPosition);
    SparkFlexConfig config = new SparkFlexConfig();
    
    config.closedLoop
      .p(0)
      .d(0)
      .outputRange(0, 0.5);

    SparkMaxConfig config_2 = new SparkMaxConfig();
    config_2.closedLoop.velocityFF(1/917);

    SparkMaxConfig config_3 = new SparkMaxConfig();
   // Set MAXMotion parameters
  config_3.closedLoop.maxMotion
    .maxVelocity(1)
    .maxAcceleration(1)
    .allowedClosedLoopError(0.5);

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
