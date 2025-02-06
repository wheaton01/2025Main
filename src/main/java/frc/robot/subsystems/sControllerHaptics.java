// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sControllerHaptics extends SubsystemBase {
  XboxController driverXbox,opXbox;
  /** Creates a new controllerHaptics. */
  public sControllerHaptics(XboxController driverXbox, XboxController opXbox) {
    this.driverXbox = driverXbox;
    this.opXbox = opXbox;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setHaptics(double setpoint){
    driverXbox.setRumble(RumbleType.kBothRumble, setpoint);
    opXbox.setRumble(RumbleType.kBothRumble, setpoint);
  }
}