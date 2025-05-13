// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Look at Gversion and version control at some point

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class LEDSubsystem extends SubsystemBase {
  CANdle candle = new CANdle(4, "DriveCanivore"); // makes a new candle with ID 0

  private final int LedCount = 300;

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
            case "colorflow":
              COLORFLOW();
              break;
            case "fire":
              FIRE();
              break;
            case "rainbow":
              RAINDOW();
              break;
            case "rgbfade":
              RGBFADE();
              break;
            case "singlefade_yellow":
              SINGLEFADE_YELLOW();
              break;
            case "strobe":
              STROBE();
              break;
            case "twinkle":
              TWINKLE();
              break;
            case "twinkleoff":
              TWINKLEOFF();
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
   
    public void COLORFLOW()
    {
      SmartDashboard.putString("Candle Colour: ", "colorflow");
      Animation ColorFlowAnimation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
      candle.animate(ColorFlowAnimation);
    }

    public void FIRE()
    {
      SmartDashboard.putString("Candle Colour: ", "fire");
      Animation FireAnimation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
      candle.animate(FireAnimation);
    }

    public void RAINDOW()
    {
      SmartDashboard.putString("Candle Colour: ", "rainbow");
      Animation RainbowAnimation = new RainbowAnimation(1, 0.1, LedCount);
      candle.animate(RainbowAnimation);
    }

    public void RGBFADE()
    {
      SmartDashboard.putString("Candle Colour: ", "rgbfade");
      Animation RGBAnimation = new RgbFadeAnimation(0.7, 0.4, LedCount);
      candle.animate(RGBAnimation);
    }
    
    public void SINGLEFADE_YELLOW()
    {
      SmartDashboard.putString("Candle Colour: ", "singlefade_yellow");
      Animation SingleFadeAnimation = new SingleFadeAnimation(255, 150, 0, 0, 0.5, LedCount);
      candle.animate(SingleFadeAnimation);
    }
    
    public void STROBE()
    {
      SmartDashboard.putString("Candle Colour: ", "strobe");
      Animation StrobeAnimation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
      candle.animate(StrobeAnimation);
    }
    
    public void TWINKLE()
    {
      SmartDashboard.putString("Candle Colour/Effect: ", "twinkle");
      Animation TwinkleAnimation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
      candle.animate(TwinkleAnimation);
    }

    public void TWINKLEOFF()
    {
      SmartDashboard.putString("Candle Colour: ", "twinkleoff");
      Animation TwinkleOffAnimation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);
      candle.animate(TwinkleOffAnimation);
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
