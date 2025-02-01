// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;

public class sElevator extends SubsystemBase {
  /** Creates a new sElevator. */
  SparkMax mElevator1, mElevator2;
  PIDController mElevatorPid;

  double PIDOutput;

  public sElevator() {
    mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    
    mElevatorPid = new PIDController(robotConstants.elevatorConstants.kP, 
                                     robotConstants.elevatorConstants.kI, 
                                     robotConstants.elevatorConstants.kD);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Position", mElevator1.getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Setpoint", mElevatorPid.getSetpoint());
    mElevatorPid.calculate(mElevator1.getEncoder().getPosition()+robotConstants.elevatorConstants.kFeedForward);
    
    
    mElevator1.set(mElevatorPid.getSetpoint());
    mElevator2.set(mElevatorPid.getSetpoint());
  }
  public void setElevatorPose(double height){
    mElevatorPid.setSetpoint(height);
  }
}
