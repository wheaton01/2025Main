// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sIntake;
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
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIntake extends Command {
  /** Creates a new setIntake. */
  boolean buseSensor;
  boolean bextend;
  double nintakeSpeed,nplaceSpeed;
  sIntake sIntake;
  public setIntake(double nintakeSpeed, sIntake sIntake, boolean buseSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.sIntake = sIntake;
    this.nintakeSpeed = nintakeSpeed; 
    this.buseSensor = buseSensor;   
    addRequirements(sIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (buseSensor) {
      if (sIntake.getCoralSensor()) {
        sIntake.setIntake(nintakeSpeed);
      } else {
        sIntake.setZero();
      }
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (buseSensor) {
      sIntake.setZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (buseSensor) {
      return sIntake.getCoralSensor();
    }
    return false;    
}
}
