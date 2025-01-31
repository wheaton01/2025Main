package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.DoubleSupplier;

public class DriveToPoseCommand extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final double speed;
  private final DoubleSupplier xOffsetSupplier;
  private final DoubleSupplier yOffsetSupplier;
  private final DoubleSupplier rotationOffsetSupplier;
  private final DoubleSupplier leftTriggerSupplier;
  private final DoubleSupplier rightTriggerSupplier;

  public DriveToPoseCommand(SwerveSubsystem swerveSubsystem, double speed, 
                            DoubleSupplier xOffsetSupplier, DoubleSupplier yOffsetSupplier, 
                            DoubleSupplier rotationOffsetSupplier, DoubleSupplier leftTriggerSupplier, 
                            DoubleSupplier rightTriggerSupplier) {
    this.swerveSubsystem = swerveSubsystem;
    this.speed = speed;
    this.xOffsetSupplier = xOffsetSupplier;
    this.yOffsetSupplier = yOffsetSupplier;
    this.rotationOffsetSupplier = rotationOffsetSupplier;
    this.leftTriggerSupplier = leftTriggerSupplier;
    this.rightTriggerSupplier = rightTriggerSupplier;
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    // Initialization code if needed
  }

  @Override
  public void execute() {
    Pose2d targetPose = getNearestAprilTagPose();
    if (targetPose == null) {
      return; // If no valid tag, do nothing
    }

    double xOffset = xOffsetSupplier.getAsDouble();
    double yOffset = yOffsetSupplier.getAsDouble();
    double rotationOffset = rotationOffsetSupplier.getAsDouble();

    Pose2d adjustedPose = new Pose2d(
      targetPose.getX() + xOffset,
      targetPose.getY() + yOffset,
      targetPose.getRotation().plus(Rotation2d.fromDegrees(rotationOffset))
    );

    swerveSubsystem.driveToPose(adjustedPose, speed);
  }

  private Pose2d getNearestAprilTagPose() {
    double leftTriggerValue = leftTriggerSupplier.getAsDouble();
    double rightTriggerValue = rightTriggerSupplier.getAsDouble();

    // Determine alliance side and get the nearest tag
    if (leftTriggerValue > rightTriggerValue) {
      return swerveSubsystem.getNearestAllianceAprilTagPose("left");
    } else {
      return swerveSubsystem.getNearestAllianceAprilTagPose("right");
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = swerveSubsystem.getPose();
    Pose2d targetPose = getNearestAprilTagPose();
    
    if (targetPose == null) {
      return true; // Stop if no valid AprilTag is found
    }

    return currentPose.equals(targetPose);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      swerveSubsystem.stop();
    }
  }
}