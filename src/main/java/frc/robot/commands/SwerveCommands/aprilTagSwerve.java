// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.poseConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.swervedrive.ApriltagRelativeRobotPose;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

public class aprilTagSwerve extends Command {
  /** Creates a new aprilTagSwerve. */


  Double tx,ty,ta,kP,kI,kD,setpoint,distToTarget, hapticSetpoint;
  DoubleSupplier vX,vY,omega;
  BooleanSupplier driveMode;
SwerveSubsystem swerve;
SwerveController controller;
CommandXboxController driverXbox, opXbox;
PIDController thetaController;
ApriltagRelativeRobotPose apriltagRelativeRobotPose;
int cameraID;
public aprilTagSwerve(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier omega,
BooleanSupplier driveMode, CommandXboxController driverXbox, CommandXboxController opXbox,ApriltagRelativeRobotPose apriltagRelativeRobotPose,int cameraID)

{
this.driverXbox = driverXbox;
this.opXbox     = opXbox;
this.swerve = swerve;
this.vX = vX;
this.vY = vY;
this.omega = omega;
this.driveMode = driveMode;
this.apriltagRelativeRobotPose = apriltagRelativeRobotPose;
this.cameraID = cameraID;
this.controller = swerve.getSwerveController();
// Use addRequirements() here to declare subsystem dependencies.
addRequirements(swerve);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(cameraID == 0)
    {
    swerve.setApriltagDrive(cameraID, swerveConstants.xOffsetReef, -swerveConstants.yOffsetReef);
    }
    else if(cameraID == 1)
    {
      swerve.setApriltagDrive(cameraID, swerveConstants.xOffsetReef, swerveConstants.yOffsetReef);

    }else if(cameraID == 2)
    {
      swerve.setApriltagDrive(cameraID, 0.0, swerveConstants.yOffsetReef);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.apriltagDrive(vX.getAsDouble(),vY.getAsDouble(),omega.getAsDouble(), cameraID);
    // distToTarget =swerve.getDistanceToTarget();
    // driverXbox.setRumble(null,hapticSetpoint = swerve.scaleDistance(distToTarget));
    // opXbox.setRumble(null,hapticSetpoint = swerve.scaleDistance(distToTarget));
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stop();
    opXbox.setRumble(RumbleType.kBothRumble,0);
    driverXbox.setRumble(RumbleType.kBothRumble,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}