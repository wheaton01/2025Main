// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sEndAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIntake extends Command {
  /** Creates a new setIntake. */
  boolean buseSensor;
  boolean bextend;
  double nspeed;
  sEndAffector sEndAffector;
  public setIntake(boolean buseSensor, boolean bextend, double nspeed, sEndAffector sEndAffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.buseSensor = buseSensor;
    this.bextend = bextend;
    this.nspeed = nspeed;
    this.sEndAffector = sEndAffector;
    
    addRequirements(sEndAffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sEndAffector.setIntake(nspeed, bextend, buseSensor);
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
