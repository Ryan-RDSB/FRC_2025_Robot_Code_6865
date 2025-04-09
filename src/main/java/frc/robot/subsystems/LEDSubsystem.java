// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.TimeUnit;


public class LEDSubsystem extends SubsystemBase {
  CANdle candle = new CANdle(0); // makes a new candle with ID 0

  private Animation m_toAnimate = null;

  public enum AnimationTypes {
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        SetAll
    }

  private AnimationTypes m_currentAnimation;
  
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
  public Command LEDCommand(String color) 
  {
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

  public Command runPattern(LEDPattern pattern) 
  {
    return run(
      () -> RED()
    );
  }

    public void RED() 
    {
      SmartDashboard.putString("Candle Colour: ", "Red");
      candle.setLEDs(255, 0, 0);
    }

    public void BLUE() 
    {
      SmartDashboard.putString("Candle Colour: ", "Blue");
      candle.setLEDs(0, 0, 255);
    }

    public void GREEN() 
    {
      SmartDashboard.putString("Candle Colour: ", "Green");
      candle.setLEDs(0, 255, 0);
    }

    public void YELLOW() 
    {
      SmartDashboard.putString("Candle Colour: ", "Yellow");
      candle.setLEDs(255, 255, 0);
    }

    public void PURPLE()
    {
      SmartDashboard.putString("Candle Colour: ", "Purple");
      candle.setLEDs(160, 32, 240);
    }

    public void ORANGE() 
    {
      SmartDashboard.putString("Candle Colour: ", "Orange");
      candle.setLEDs(255, 165, 0);
    }

    public void OFF() 
    {
      SmartDashboard.putString("Candle Colour: ", "Off");
      candle.setLEDs(255, 255, 255);
    }

  public CANdle getCaNdle()
  {
    return candle;
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
