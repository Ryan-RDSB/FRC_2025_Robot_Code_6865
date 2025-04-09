// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//test
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//logging Advantage Kit
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.LogFileUtil;
import edu.wpi.first.wpilibj.RobotBase;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;



import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.TimedRobot;
//

public class Robot extends LoggedRobot {

  
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /*public Robot() {
    /*m_robotContainer = new RobotContainer();

      Logging metadata
  
      Logger.recordMetadata("ProjectName", "6865LogFile");
  
      if (RobotBase.isReal()) {
          Logger.addDataReceiver(new WPILOGWriter()); // Log to USB
          Logger.addDataReceiver(new NT4Publisher()); // Live data
      } else {
          setUseTiming(false); // Run sim fast
          Logger.addDataReceiver(new WPILOGWriter("logs/sim")); // New sim log
          Logger.addDataReceiver(new NT4Publisher());
      }
  
      Logger.start(); 
      // Must be last */

      public Robot() {
        m_robotContainer = new RobotContainer();
    
        // Logging metadata
        Logger.recordMetadata("ProjectName", "6865LogFile");
    
        if (RobotBase.isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to USB
            Logger.addDataReceiver(new NT4Publisher()); // Live NT data
        } else {
            setUseTiming(false); // Fast sim
            Logger.addDataReceiver(new WPILOGWriter("logs/sim")); // Save to logs/sim
            Logger.addDataReceiver(new NT4Publisher()); // Allow AdvantageScope to connect
        }
    
        Logger.start(); // Must be last
    
 }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 

    double time = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    Logger.recordOutput("Test/SineWave", Math.sin(time));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
