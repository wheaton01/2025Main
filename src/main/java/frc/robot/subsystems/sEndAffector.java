// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.intakeConstants;
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
public class sEndAffector extends SubsystemBase {
  /** Creates a new sEndAffector. */
  VictorSPX mPlace;

  //
  AnalogInput aIntakeSensor; 
  AnalogInput aPlaceSensor;
  CANBus rioBus;
  public sEndAffector() {
    // mIntake = new SparkMax(Constants.robotConstants.kintakeSparkID, MotorType.kBrushed);
    // mPlace =  new SparkMax(Constants.robotConstants.kPlacesparkID , MotorType.kBrushed);
    mPlace = new  VictorSPX(robotConstants.kPlacesparkID);
    
    }

  @Override
  public void periodic() {
  }


  public void setBallIntake(){
    mPlace.set(VictorSPXControlMode.PercentOutput,intakeConstants.kBallIntakeSpeed);
  }
  public void setPlace(){
    mPlace.set(VictorSPXControlMode.PercentOutput,intakeConstants.kPlaceSpeed);
  }

  public void setZero(){
    mPlace.set(VictorSPXControlMode.PercentOutput,0);
  }



  public boolean hasCoral(){
    return aIntakeSensor.getValue()>Constants.robotConstants.kintakeSensorThreshold;
  }
}

