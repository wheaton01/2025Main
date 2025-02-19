// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.crypto.SealedObject;

import javax.crypto.SealedObject;

import edu.wpi.first.wpilibj.AnalogInput;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.robotConstants;

public class sEndAffector extends SubsystemBase {
  /** Creates a new sEndAffector. */
  VictorSPX mIntake;
  VictorSPX mPlace;

  //
  AnalogInput aIntakeSensor; 
  AnalogInput aPlaceSensor;
  CANBus rioBus;
  public sEndAffector() {
    // mIntake = new SparkMax(Constants.robotConstants.kintakeSparkID, MotorType.kBrushed);
    // mPlace =  new SparkMax(Constants.robotConstants.kPlacesparkID , MotorType.kBrushed);
    rioBus = new CANBus("rio");

    mIntake = new VictorSPX(robotConstants.kintakeSparkID);
    mPlace = new  VictorSPX(robotConstants.kPlacesparkID);
    
    aIntakeSensor = new AnalogInput(Constants.robotConstants.kIntakeSensorID);
      //aPlaceSensor = new  AnalogInput(Constants.robotConstants.kPlaceSensorID); 
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake sensor val",aIntakeSensor.getValue());
    // This method will be called once per scheduler run
  }

  public void setIntake(double nintakeSpeed,double nplaceSpeed){
      mIntake.set(VictorSPXControlMode.PercentOutput,nintakeSpeed);
      mPlace.set(VictorSPXControlMode.PercentOutput,nplaceSpeed);
    }
  
  public void setPlace(double nspeed){
    mPlace.set(VictorSPXControlMode.PercentOutput,nspeed);
  }
  public void setIntake(double nspeed){
    mIntake.set(VictorSPXControlMode.PercentOutput,nspeed);
  }
  public void setZero(){
    mIntake.set(VictorSPXControlMode.PercentOutput,0);
    mPlace.set(VictorSPXControlMode.PercentOutput,0);
  }
  public boolean getCoralSensor(){
    if (aIntakeSensor.getValue()>Constants.robotConstants.kintakeSensorThreshold){
      return true;
    } else {
      return false;
      
    }
  }


  public boolean hasCoral(){
    return aIntakeSensor.getValue()>Constants.robotConstants.kintakeSensorThreshold;
  }
}

