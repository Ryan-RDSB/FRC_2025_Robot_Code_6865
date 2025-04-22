// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbSubsystem extends SubsystemBase {
  // Initialize the motor (Flex/MAX are setup the same way)
SparkFlex climb1 = new SparkFlex(14, MotorType.kBrushless);
SparkFlex climb2 = new SparkFlex(15, MotorType.kBrushless);


  /** Creates a new Subsystem. */
  public ClimbSubsystem() 
  {
    SparkMaxConfig config4 = new SparkMaxConfig();
    SparkMaxConfig config5 = new SparkMaxConfig();

    config4.idleMode(IdleMode.kBrake);
    config5.idleMode(IdleMode.kBrake);

    config5.follow(14, true);

    climb1.configure(config4, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climb2.configure(config5, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  /**
   * Sets motor controllers to run-to-pos based off distance
   *
   * @return a command
   */
  public Command ClimbCommand(double speed) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          runClimber(speed);
        });
  }

  public void runClimber(double speed)
  {
    climb1.set(speed);
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
