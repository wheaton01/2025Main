// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sClimber extends SubsystemBase {
  /** Creates a new sClimber. */
  Solenoid sClimb,sdeployClimb;

  boolean enableClimber = false;

  public sClimber() {
    sClimb =       new Solenoid(PneumaticsModuleType.REVPH, Constants.robotConstants.climberConstants.kClimberSolenoidID);  
    sdeployClimb = new Solenoid(PneumaticsModuleType.REVPH, Constants.robotConstants.climberConstants.kDeployClimberSolenoidID);    
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(getName(), enableClimber);
    SmartDashboard.putBoolean("Climber Deployed", sdeployClimb.get());
    // This method will be called once per scheduler run
  }
  public void deployClimber(boolean bdeploy){
    sdeployClimb.set(bdeploy);
  }
  public void climb(boolean bclimb){
    sClimb.set(bclimb);
  } 
  public void disableSafety(){
    enableClimber = true;
  }

}
