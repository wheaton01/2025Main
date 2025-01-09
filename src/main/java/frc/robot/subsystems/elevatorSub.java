// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
public class elevatorSub extends SubsystemBase {
  /** Creates a new elevatorSub. */
  SparkMax elevatorSpark1,elevatorSpark2;
  public elevatorSub() {
    elevatorSpark1 = new SparkMax(robotConstants.kelevatorSparkID1, MotorType.kBrushless);
    elevatorSpark2 = new SparkMax(robotConstants.kelevatorSparkID2, MotorType.kBrushless);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
