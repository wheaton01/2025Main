// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.SealedObject;

import javax.crypto.SealedObject;

import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.robotConstants;

public class sEndAffector extends SubsystemBase {
  PneumaticHub pHub;
  /** Creates a new sEndAffector. */
  Solenoid sEAExtend;
  TalonFX mIntake;
  TalonFX mPlace;

  //
  AnalogInput aIntakeSensor; 
  AnalogInput aPlaceSensor;
  CANBus rioBus;
  public sEndAffector() {
    // mIntake = new SparkMax(Constants.robotConstants.kintakeSparkID, MotorType.kBrushed);
    // mPlace =  new SparkMax(Constants.robotConstants.kPlacesparkID , MotorType.kBrushed);
    rioBus = new CANBus("rio");

    mIntake = new TalonFX(robotConstants.kintakeSparkID, rioBus);
    mPlace = new TalonFX(robotConstants.kPlacesparkID, rioBus);
    
    sEAExtend = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.robotConstants.kPneuExtendID);

    aIntakeSensor = new AnalogInput(Constants.robotConstants.kIntakeSensorID);
      aPlaceSensor = new  AnalogInput(Constants.robotConstants.kPlaceSensorID); 
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntake(double nintakeSpeed,double nplaceSpeed){
      mIntake.set(nintakeSpeed);
      mPlace.set(nplaceSpeed);
    }
  
  public void setPlace(double nspeed){
    mPlace.set(nspeed);
  }
  public void setIntake(double nspeed){
    mIntake.set(nspeed);
  }
  public void setZero(){
    mIntake.set(0);
    mPlace.set(0);
  }
  public boolean getCoralSensor(){
    if (aPlaceSensor.getValue()>Constants.robotConstants.kintakeSensorThreshold){
      return true;
    } else {
      return false;
      
    }
  }

  public void setExtend(boolean bextend){
    sEAExtend.set(bextend);
  }
  public boolean hasCoral(){
    return aIntakeSensor.getValue()>Constants.robotConstants.kintakeSensorThreshold;
  }
}

