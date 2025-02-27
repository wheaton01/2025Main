package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.poseConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

public class DriveToPoseCommand extends Command {
  // Subsystem for controlling the swerve drive
  private final SwerveSubsystem swerveSubsystem;

  // Speed at which the robot should drive
  @SuppressWarnings("unused")
  private final double speed;

  // Suppliers for offsets and trigger inputs (dynamically received from the controller)
  private final DoubleSupplier xOffsetSupplier;
  private final DoubleSupplier yOffsetSupplier;
  private final DoubleSupplier rotationOffsetSupplier;
  private final DoubleSupplier leftTriggerSupplier;
  private final DoubleSupplier rightTriggerSupplier;
  
  private boolean bInAuton = false;
  private boolean bhumanPlayerStation = false;
  public Pose2d adjustedPose;
  private Command pathFindingCommand; // Command to drive to the target pose
  Pose2d targetPose;
  // Constructor to initialize the command
  public DriveToPoseCommand(SwerveSubsystem swerveSubsystem, double speed, 
                            DoubleSupplier xOffsetSupplier, DoubleSupplier yOffsetSupplier, 
                            DoubleSupplier rotationOffsetSupplier, 
                            DoubleSupplier rightTriggerSupplier,
                             DoubleSupplier leftTriggerSupplier, boolean bhumanPlayerStation, boolean bInAuton) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = speed;
    this.xOffsetSupplier = xOffsetSupplier;
    this.yOffsetSupplier = yOffsetSupplier;
    this.rotationOffsetSupplier = rotationOffsetSupplier;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
    this.bhumanPlayerStation = bhumanPlayerStation;
    this.bInAuton = bInAuton;
    
    // Declare subsystem dependencies so that no other command interferes
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(getSubsystem(), "Now Driving To nearest pose");

    // Get the closest AprilTag pose for the reef
    if(!bhumanPlayerStation){
    targetPose = swerveSubsystem.getNearestReefAprilTagPose();
    SmartDashboard.putString("DistToReefTag", targetPose.getX() + " " + targetPose.getY());
    if(rightTriggerSupplier.getAsDouble()<.2){
    adjustedPose = calculateOffsetPose(targetPose,poseConstants.xOffsetReef,-poseConstants.yOffsetReef);
    }
    if(leftTriggerSupplier.getAsDouble()<.2){
    adjustedPose = calculateOffsetPose(targetPose,poseConstants.xOffsetReef,poseConstants.yOffsetReef);
    }
  }

    if (bhumanPlayerStation) {
      targetPose = getHPStation();
      adjustedPose = calculateHPpose(targetPose, poseConstants.xOffsetHPStation,poseConstants.yOffsetHPStation);
    }

    // If no valid tag is found, exit early to avoid errors
    if (targetPose == null) {
      return;
    }

    swerveSubsystem.setTargetPose(adjustedPose);

    // Create the pathfinding command to move the robot to the adjusted pose
    pathFindingCommand = swerveSubsystem.driveToPose(adjustedPose);

    // Schedule the pathfinding command
    pathFindingCommand.schedule();
  }

  @Override
  public void execute() {
    // Pathfinding command is already running, no need to implement custom logic here
  }

  private Pose2d getHPStation() {
    return swerveSubsystem.getNearestHumanPlayerTagPose();
  }

  @Override
  public boolean isFinished() {
    // Check if the pathfinding command is finished
    if (leftTriggerSupplier.getAsDouble()+leftTriggerSupplier.getAsDouble()<.1){
    return true;
  }
    return pathFindingCommand.isFinished();
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends or is interrupted
    swerveSubsystem.stop();
    if (interrupted) {
      swerveSubsystem.stop();
    }
  }
  public Pose2d calculateOffsetPose(Pose2d tagPose, double dF, double dS) {
    double xT = tagPose.getX();
    double yT = tagPose.getY();
    double thetaT = tagPose.getRotation().getRadians();

    // Compute new x, y
    double xR = xT + dF * Math.cos(thetaT) - dS * Math.sin(thetaT);
    double yR = yT + dF * Math.sin(thetaT) + dS * Math.cos(thetaT);
    
    // Compute new heading (180 degrees flipped)
    Rotation2d thetaR = tagPose.getRotation().rotateBy(Rotation2d.fromDegrees(180));

    return new Pose2d(xR, yR, thetaR);
}
public Pose2d calculateHPpose(Pose2d tagPose, double dF, double dS) {
  double xT = tagPose.getX();
  double yT = tagPose.getY();
  double thetaT = tagPose.getRotation().getRadians();

  // Compute new x, y
  double xR = xT + dF * Math.cos(thetaT) - dS * Math.sin(thetaT);
  double yR = yT + dF * Math.sin(thetaT) + dS * Math.cos(thetaT);
  
  // Keep the same heading as the tag
  Rotation2d thetaR = tagPose.getRotation();

  return new Pose2d(xR, yR, thetaR);
}

}
