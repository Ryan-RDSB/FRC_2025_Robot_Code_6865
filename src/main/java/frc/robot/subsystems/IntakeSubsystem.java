// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class IntakeSubsystem extends SubsystemBase {
  private LaserCan lc;
  public boolean coralIn;
  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {
    lc = new LaserCan(0);
    try {
      lc.setRangingMode(LaserCan.RangingMode.SHORT);
      lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 1000));
      lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command IntakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          //Stuff goes here
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public int getLaserMeasurement() {
     LaserCan.Measurement measurement = lc.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return measurement.distance_mm;
    }
    return -1;
  }

  @Override
  public void periodic() {
    int measurement = getLaserMeasurement();
    
    if (measurement > 30 & measurement < 50)
    {
      coralIn = true;
    }
    else
    {
      coralIn = false;
    }
    //System.out.println("Coral In: " + coralIn);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LaserCAN Reading:", measurement);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
