// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.robotConstants.climberConstants;
/* 
╔══════════════════════════════════════════════════════════════════════════════════════╗
║  __/\\\\\\\\\\\\\\\___/\\\\\\\\\\\\\\\____/\\\\\\\\\\\\\\\______/\\\\\\\\\\_________ ║
║  _\/////////////\\\__\/\\\///////////____\/\\\///////////_____/\\\///////\\\________ ║
║   ____________/\\\/___\/\\\_______________\/\\\_______________\///______/\\\________ ║
║   __________/\\\/_____\/\\\\\\\\\\\\______\/\\\\\\\\\\\\_____________/\\\//_________ ║
║    ________/\\\/_______\////////////\\\____\////////////\\\__________\////\\\_______ ║
║     ______/\\\/____________________\//\\\______________\//\\\____________\//\\\_____ ║
║      ____/\\\/___________/\\\________\/\\\___/\\\________\/\\\___/\\\______/\\\_____ ║
║       __/\\\/____________\//\\\\\\\\\\\\\/___\//\\\\\\\\\\\\\/___\///\\\\\\\\\/_____ ║
║        _\///_______________\/////////////______\/////////////_______\/////////______ ║
╚══════════════════════════════════════════════════════════════════════════════════════╝
*/
public class sClimber extends SubsystemBase {
  /** Creates a new sClimber. */
  Solenoid sClimb,sdeployClimb,sdropRamp;

  boolean enableClimber = false;
  boolean bgoFurther = false;
  SparkMax climberMotor;
  RelativeEncoder climberEncoder;
  boolean bHasExtended = false;
  public sClimber() {
    climberMotor = new SparkMax(climberConstants.kMotorID,MotorType.kBrushless);
    sClimb =       new Solenoid(1,PneumaticsModuleType.CTREPCM, Constants.robotConstants.climberConstants.kClimb2ID);  
    sdeployClimb = new Solenoid(1,PneumaticsModuleType.CTREPCM, Constants.robotConstants.climberConstants.kClimb1ID);    
    sdropRamp =    new Solenoid(1,PneumaticsModuleType.CTREPCM, Constants.robotConstants.climberConstants.kRamp1ID);    
    climberEncoder = climberMotor.getEncoder();
    climberEncoder.setPosition(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean(getName(), enableClimber);
    SmartDashboard.putNumber("Climb Encoder Value", climberEncoder.getPosition());
   // SmartDashboard.putBoolean("Climber Deployed", sdeployClimb.get());
    // This method will be called once per scheduler run
  }
  public void deployClimber(){
    if (enableClimber) {
    //sdeployClimb.set(true);
    }
  } 
  public void stowClimber(){
    if (enableClimber) {

    //sdeployClimb.set(false);
    }
  }
  public void climb(){
    if (enableClimber) {
      if (bHasExtended && climberEncoder.getPosition() > climberConstants.kClimbMax) {
        climberMotor.set(climberConstants.kClimbSpeed);
      }
      if (climberEncoder.getPosition() <= climberConstants.kClimbMax && climberEncoder.getPosition() > climberConstants.kClimbSafety) {
        climberMotor.set(climberConstants.kClimbSpeed*.5);
      }
   }
  }  
  public void unClimb(){
    if (enableClimber ) {

      bHasExtended = true;
      if (climberEncoder.getPosition() > climberConstants.kExtendedPose) {
        climberMotor.set(-climberConstants.kClimbSpeed*.25);
      }else{
      climberMotor.set(-climberConstants.kClimbSpeed);
      }
    }
    
  } 
  public void zeroClimb(){
    climberMotor.set(0);
  }
  public void disableSafety(){
    enableClimber = true;
  }
  public void dropRamp(){
    sdropRamp.set(enableClimber);
  }
  

}
