// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.seqComs;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.subsystems.sEndAffector;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.intakeConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class intakeCoral extends SequentialCommandGroup {
  /** Creates a new intakeCoral. */
  public intakeCoral(sEndAffector m_endAffector) {
    //this code should intake the coral all the way until its partially inside the intake
    addCommands(new setIntake(true, false, intakeConstants.kIntakeSpeed,0.0, m_endAffector),
                new setIntake(false,false, intakeConstants.kIntakeSpeed*.5,intakeConstants.kPlaceSpeed*.5, m_endAffector).withTimeout(.25),
                new setIntake(false,false, 0.0,0.0, m_endAffector));
  }
}
