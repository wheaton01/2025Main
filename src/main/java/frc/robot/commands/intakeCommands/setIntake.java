// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sEndAffector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setIntake extends Command {
  /** Creates a new setIntake. */
  boolean buseSensor;
  boolean bextend;
  double nintakeSpeed,nplaceSpeed;
  sEndAffector sEndAffector;
  public setIntake(boolean buseSensor, boolean bextend, double nintakeSpeed,double nplaceSpeed, sEndAffector sEndAffector) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.buseSensor = buseSensor;
    this.bextend = bextend;
    this.nintakeSpeed = nintakeSpeed;
    this.nplaceSpeed = nplaceSpeed;
    this.sEndAffector = sEndAffector;
    
    addRequirements(sEndAffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(buseSensor && !sEndAffector.getCoralSensor()){

      sEndAffector.setIntake(nintakeSpeed,nplaceSpeed);
    }
    if(!buseSensor){
    sEndAffector.setIntake(nintakeSpeed,nplaceSpeed);
    }

    sEndAffector.setExtend(bextend);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(buseSensor){
    sEndAffector.setZero();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (buseSensor){
      return sEndAffector.getCoralSensor();
    }
    return false;
    
}
}
