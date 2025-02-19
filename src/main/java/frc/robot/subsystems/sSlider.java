// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sSlider extends SubsystemBase {
  /** Creates a new sSlider. */
    PneumaticHub pneuHub;
    Solenoid sEAExtend;

  public sSlider() {
    pneuHub = new PneumaticHub(3);
    sEAExtend = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.robotConstants.kPneuExtendID);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setExtend(boolean bextend){
    sEAExtend.set(bextend);
  }
}
