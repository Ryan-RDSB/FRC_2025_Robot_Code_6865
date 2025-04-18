// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkMax motor3 = new SparkMax(13, MotorType.kBrushless);


  /** Creates a new Subsystem. */
  public ClawSubsystem() 
  {
    
    SparkMaxConfig config3 = new SparkMaxConfig();
    config3
    .inverted(true).idleMode(IdleMode.kBrake);

    // Apply configs - reset old parameters, and persist through power-cycles. 
    motor3.configure(config3, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command ClawCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          runClaw(speed);
        });
  }

  public void runClaw(double speed)
  {
    motor3.set(speed);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean ClawCondition() {
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
