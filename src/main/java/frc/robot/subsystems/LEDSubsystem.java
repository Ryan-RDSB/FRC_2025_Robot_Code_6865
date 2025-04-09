package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  private final IntakeSubsystem intake;
  private boolean wasCoralIn = false;
  private double coralInStartTime = 0.0;
  private final int kLength = 60;

  public LEDSubsystem(IntakeSubsystem intake) {
    this.intake = intake;
    m_led = new AddressableLED(9);
    m_buffer = new AddressableLEDBuffer(kLength);
    m_led.setLength(kLength);
    m_led.start();
  }

  @Override
  public void periodic() {
    boolean isCoralIn = intake.coralIn;
    double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    if (isCoralIn && !wasCoralIn) {
      coralInStartTime = time;
    }

    if (isCoralIn && time - coralInStartTime < 3.0) {
      // Green for 3 seconds
      for (int i = 0; i < kLength; i++) {
        m_buffer.setRGB(i, 0, 255, 0);
      }
      SmartDashboard.putString("LED State", "Green");
    } else if (isCoralIn) {
      // Yellow/Black scrolling animation
      double scrollSpeed = 10.0; // LEDs per second
      int offset = (int)(time * scrollSpeed) % kLength;
      for (int i = 0; i < kLength; i++) {
        if (((i + offset) / 5) % 2 == 0) {
          m_buffer.setRGB(i, 255, 255, 0); // Yellow
        } else {
          m_buffer.setRGB(i, 0, 0, 0); // Off/Black
        }
      }
      SmartDashboard.putString("LED State", "Yellow/Black Scroll");
    } else {
      // Default off
      for (int i = 0; i < kLength; i++) {
        m_buffer.setRGB(i, 0, 0, 0);
      }
      SmartDashboard.putString("LED State", "Off");
    }

    wasCoralIn = isCoralIn;
    m_led.setData(m_buffer);
  }

  // -------------------------
  // New method to fix the error
  // -------------------------
  public Command LEDCommand(String color) {
    return this.run(() -> {
      switch (color.toLowerCase()) {
        case "red":
          for (int i = 0; i < kLength; i++) {
            m_buffer.setRGB(i, 255, 0, 0);
          }
          break;
        case "green":
          for (int i = 0; i < kLength; i++) {
            m_buffer.setRGB(i, 0, 255, 0);
          }
          break;
        case "blue":
          for (int i = 0; i < kLength; i++) {
            m_buffer.setRGB(i, 0, 0, 255);
          }
          break;
        default:
          for (int i = 0; i < kLength; i++) {
            m_buffer.setRGB(i, 0, 0, 0);
          }
          break;
      }
      SmartDashboard.putString("LED Manual Command", color);
      m_led.setData(m_buffer);
    });
  }
}
