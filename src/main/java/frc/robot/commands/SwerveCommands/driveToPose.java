package frc.robot.commands.SwerveCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class driveToPose extends Command {
  private final SwerveSubsystem swerve;
  private final Pose2d pose;
  private Command pathFindingCommand;

  /** Creates a new driveToPose. */
  public driveToPose(SwerveSubsystem swerve, Pose2d pose) {
    this.swerve = swerve;
    this.pose = pose;

    // Declare the subsystem dependency
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Create the pathfinding command using the swerve drive's driveToPose method
    pathFindingCommand = swerve.driveToPose(pose);
    
    // Schedule the pathfinding command to run
    pathFindingCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // The pathfinding command is already running, so no need to do anything here
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Cancel the pathfinding command if interrupted or finished
    pathFindingCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathFindingCommand.isFinished(); // This will end when the pathfinding command is finished
  }
}
