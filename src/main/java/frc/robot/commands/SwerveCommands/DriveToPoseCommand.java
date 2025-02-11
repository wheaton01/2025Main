package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.aprilTagConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;

public class DriveToPoseCommand extends Command {
  // Subsystem for controlling the swerve drive
  private final SwerveSubsystem swerveSubsystem;
  
  // Speed at which the robot should drive
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
                            DoubleSupplier rotationOffsetSupplier, DoubleSupplier leftTriggerSupplier, 
                            DoubleSupplier rightTriggerSupplier,boolean bhumanPlayerStation, boolean bInAuton) {
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
    // Runs once when the command starts. Currently does nothing but can be used for setup.
  }

  @Override
  public void execute() {
    // Get the closest AprilTag pose for the reef
    Pose2d targetPose = getNearestReefTagPose();
    if (bhumanPlayerStation){
      targetPose = getHPStation();
    }
    
    // If no valid tag is found, exit early to avoid errors
    if (targetPose == null) {
      return;
    }
  
    // Get predefined offsets from constants
    double leftOffset  = aprilTagConstants.klReefOffset;
    double rightOffset = aprilTagConstants.krReefOffset;
  
    // Get the dynamic offset values from the controller inputs
    double xOffset = xOffsetSupplier.getAsDouble();
    double yOffset = yOffsetSupplier.getAsDouble();
    double rotationOffset = rotationOffsetSupplier.getAsDouble();
  
    // Get the trigger values (scale between 0 and 1)
    double leftTrigger = leftTriggerSupplier.getAsDouble();
    double rightTrigger = rightTriggerSupplier.getAsDouble();
  
    // You can choose to take the average of both triggers or scale the speed based on whichever trigger is pressed more
    double triggerScale = (leftTrigger + rightTrigger) / 2; // Scale the speed between 0 and 1
  
    // Adjust the speed based on the trigger value
    double scaledSpeed = speed * triggerScale;
  
    // Create a new target pose with the adjusted offsets
    adjustedPose = new Pose2d(
      targetPose.getX() + xOffset,
      targetPose.getY() + yOffset,
      targetPose.getRotation().plus(Rotation2d.fromDegrees(rotationOffset))
    );
    swerveSubsystem.setTargetPose(adjustedPose);
  
    // Command the swerve drive system to move to the adjusted pose at the scaled speed
    swerveSubsystem.driveToPose(adjustedPose, scaledSpeed);
  }
  
  
  /**
   * Determines which reef AprilTag to use for navigation.
   * Uses the controller triggers to determine if the robot should offset left or right.
   */
  private Pose2d getNearestReefTagPose() {
      if (leftTriggerSupplier.getAsDouble() > rightTriggerSupplier.getAsDouble()) {
          return swerveSubsystem.getNearestReefAprilTagPose("left"); // Offset left
      }
      if (leftTriggerSupplier.getAsDouble() <= rightTriggerSupplier.getAsDouble()) {
          return swerveSubsystem.getNearestReefAprilTagPose("right"); // Offset right
      }
      
      // Default case: Center robot to tag. for algae!
      return swerveSubsystem.getNearestReefAprilTagPose("center");
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
