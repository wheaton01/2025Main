package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
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

  // Constructor to initialize the command
  public DriveToPoseCommand(SwerveSubsystem swerveSubsystem, double speed, 
                            DoubleSupplier xOffsetSupplier, DoubleSupplier yOffsetSupplier, 
                            DoubleSupplier rotationOffsetSupplier, 
                            DoubleSupplier rightTriggerSupplier,
                             DoubleSupplier leftTriggerSupplier,boolean bhumanPlayerStation, boolean bInAuton) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = speed;
    this.xOffsetSupplier = xOffsetSupplier;
    this.yOffsetSupplier = yOffsetSupplier;
    this.rotationOffsetSupplier = rotationOffsetSupplier;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
    
    // Declare subsystem dependencies so that no other command interferes
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    SmartDashboard.putString(getSubsystem(), "Now Driving To nearest pose");
    // Runs once when the command starts. Currently does nothing but can be used for setup.
  }

  @Override
  public void execute() {
    // Get the closest AprilTag pose for the reef
    Pose2d targetPose = getNearestReefTagPose();
    SmartDashboard.putString("DistToReefTag",targetPose.getX()+" " +targetPose.getY());
    if (bhumanPlayerStation){
      targetPose = getHPStation();
    }
    
    // If no valid tag is found, exit early to avoid errors
    if (targetPose == null) {
      return;
    }
  
    // Get predefined offsets from constants
    // double leftOffset  = aprilTagConstants.klReefOffset;
    // double rightOffset = aprilTagConstants.krReefOffset;
  
    // Get the dynamic offset values from the controller inputs
    double xOffset = xOffsetSupplier.getAsDouble();
    double yOffset = yOffsetSupplier.getAsDouble();
    double rotationOffset = rotationOffsetSupplier.getAsDouble();
  
    // Get the trigger values (scale between 0 and 1)
    double leftTrigger = leftTriggerSupplier.getAsDouble();
    double rightTrigger = rightTriggerSupplier.getAsDouble();
  
    // You can choose to take the average of both triggers or scale the speed based on whichever trigger is pressed more
    double triggerScale = Math.max(leftTrigger, rightTrigger); // Use the highest trigger value
    @SuppressWarnings("unused")//this stuff was mostly used when using more custom code for pathing, pathplanner should handle this
    double scaledSpeed = triggerScale; // Scale the speed accordingly
    
    // Create a new target pose with the adjusted offsets
    adjustedPose = new Pose2d(
      targetPose.getX() + xOffset,
      targetPose.getY() + yOffset,
      targetPose.getRotation().plus(Rotation2d.fromDegrees(rotationOffset))
    );
    swerveSubsystem.setTargetPose(adjustedPose);
  
    swerveSubsystem.driveToPose(adjustedPose);
  }
  
  
  /**
   * Determines which reef AprilTag to use for navigation.
   * Uses the controller triggers to determine if the robot should offset left or right.
   */
  private Pose2d getNearestReefTagPose() {
      if (leftTriggerSupplier.getAsDouble() > rightTriggerSupplier.getAsDouble()) {
          return swerveSubsystem.getNearestReefAprilTagPose(-swerveConstants.sideOffsetDistance,swerveConstants.kforwardOffsetDistance); // Offset left
      }
      if (leftTriggerSupplier.getAsDouble() <= rightTriggerSupplier.getAsDouble()) {
          return swerveSubsystem.getNearestReefAprilTagPose(swerveConstants.sideOffsetDistance,swerveConstants.kforwardOffsetDistance); // Offset right
      }
      
      // Default case: Center robot to tag. for algae!
      return swerveSubsystem.getNearestReefAprilTagPose(0,swerveConstants.kforwardOffsetDistance);

  }
  private Pose2d getHPStation(){
    return swerveSubsystem.getNearestHumanPlayerTagPose();
  }

  @Override
  public boolean isFinished() {
    // Check if both triggers are back to zero
    if (leftTriggerSupplier.getAsDouble() == 0 && rightTriggerSupplier.getAsDouble() == 0 && !bInAuton) {
      return true; // Stop the command when both triggers are zero
    }
    if (bInAuton){
      return swerveSubsystem.isAtPose();
    }
    // Otherwise, keep the command running
    return false;
  }
  

  @Override
  public void end(boolean interrupted) {
    // Stop the robot when the command ends or is interrupted
    if (bInAuton){
      swerveSubsystem.stop();
    }
    if (interrupted) {
      swerveSubsystem.stop();
    }
  }
}
