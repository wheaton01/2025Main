// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.sControllerHaptics;
import frc.robot.subsystems.sEndAffector;
import frc.robot.subsystems.sIntake;
import frc.robot.Constants.robotConstants.intakeConstants;
import frc.robot.commands.setCHaptics;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeCoral extends SequentialCommandGroup {
  /** Creates a new intakeCoral. */
  // This command should intake the coral all the way until its partially inside the intake
  // It should also use the haptic feedback to indicate when the coral is inside the intake
  sEndAffector m_endAffector;
  sControllerHaptics m_controllerHaptics;
  sIntake m_intake;
  public intakeCoral(sEndAffector m_endAffector,sControllerHaptics m_controllerHaptics, sIntake m_intake) {
    //this code should intake the coral all the way until its partially inside the intake
    this.m_endAffector = m_endAffector;
    this.m_controllerHaptics = m_controllerHaptics; 
    this.m_intake = m_intake;
    addCommands(new SequentialCommandGroup(
      new ParallelCommandGroup(
        new setIntake(intakeConstants.kIntakeSpeed, m_intake, false),
        new InstantCommand(m_endAffector::setPlace).withTimeout(.5),
        new setCHaptics(m_controllerHaptics, 0.8).withTimeout(.5)).withTimeout(.5),
    new WaitCommand(0),
    new setIntake(0, m_intake,false).withTimeout(0),
    new InstantCommand(m_endAffector::setZero).withTimeout(0)));
  }
}
