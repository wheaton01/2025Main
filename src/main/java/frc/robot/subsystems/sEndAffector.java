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
import frc.robot.Constants.robotConstants.intakeConstants;

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

