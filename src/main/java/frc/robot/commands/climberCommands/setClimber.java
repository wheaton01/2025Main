// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climberCommands;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.sClimber;

public class setClimber extends Command {
  /** Creates a new setClimber. */
  sClimber sClimber;
  boolean bdeploy, bclimb;

  public setClimber(sClimber sClimber, boolean bdeploy, boolean bclimb) {
    this.sClimber = sClimber;
    this.bdeploy = bdeploy;
    this.bclimb = bclimb;
    // Use addRequirements() here to declare subsystem dependencies.

  addRequirements(sClimber);
    }

    @Override
    public void initialize() {
      sClimber.deployClimber(bdeploy);
      sClimber.climb(bclimb);
      }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
