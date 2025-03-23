// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class LEDSubsystem extends SubsystemBase {
  CANdle candle = new CANdle(4, "DriveCanivore"); // makes a new candle with ID 0
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() 
  {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    candle.configAllSettings(config);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command LEDCommand(String color) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          switch (color){
            case "red":
             RED();
              break;
            case "green":
             GREEN();
              break;
            case "blue":
              BLUE();
              break;
            case "yellow":
              YELLOW();
              break;
            case "purple":
              PURPLE();
              break;
            case "orange":
              ORANGE();
              break;
            default:
              OFF();
              break;
          }
        });
  }

    public void RED() {
      candle.setLEDs(255, 0, 0);
    }

    public void BLUE() {
      candle.setLEDs(0, 0, 255);
    }

    public void GREEN() {
      candle.setLEDs(0, 255, 0);
    }

    public void YELLOW() {
      candle.setLEDs(255, 255, 0);
    }
    public void PURPLE(){
      candle.setLEDs(160, 32, 240);
    }
    public void ORANGE() {
      candle.setLEDs(255, 165, 0);
    }
    public void OFF() {
      candle.setLEDs(255, 255, 255);
    }
  /*
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
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
