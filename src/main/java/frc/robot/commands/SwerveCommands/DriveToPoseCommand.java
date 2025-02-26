package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
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
    Pose2d targetPose = getNearestReefTagPose();
    SmartDashboard.putString("DistToReefTag", targetPose.getX() + " " + targetPose.getY());

    if (bhumanPlayerStation) {
      targetPose = getHPStation();
    }

    // If no valid tag is found, exit early to avoid errors
    if (targetPose == null) {
      return;
    }

    // Get dynamic offset values from the controller inputs
    double xOffset = xOffsetSupplier.getAsDouble();
    double yOffset = yOffsetSupplier.getAsDouble();
    double rotationOffset = rotationOffsetSupplier.getAsDouble();

    // Adjust the target pose based on the dynamic offsets
    adjustedPose = new Pose2d(
      targetPose.getX() + xOffset,
      targetPose.getY() + yOffset,
      targetPose.getRotation().plus(Rotation2d.fromDegrees(rotationOffset))
    );

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

  private Pose2d getNearestReefTagPose() {
    if (leftTriggerSupplier.getAsDouble() > rightTriggerSupplier.getAsDouble()) {
      return swerveSubsystem.getNearestReefAprilTagPose(-swerveConstants.ksideOffsetDistance, swerveConstants.kforwardOffsetDistance); // Offset left
    }
    if (leftTriggerSupplier.getAsDouble() <= rightTriggerSupplier.getAsDouble()) {
      return swerveSubsystem.getNearestReefAprilTagPose(swerveConstants.ksideOffsetDistance, swerveConstants.kforwardOffsetDistance); // Offset right
    }

    // Default case: Center robot to tag
    return swerveSubsystem.getNearestReefAprilTagPose(0, swerveConstants.kforwardOffsetDistance);
  }

  private Pose2d getHPStation() {
    return swerveSubsystem.getNearestHumanPlayerTagPose(0, swerveConstants.kHPForwardOffsetDistance);
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
}
