package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class sSlider extends SubsystemBase {
  /** Creates a new sSlider. */
  private final Solenoid sEAExtend;

  public sSlider() {
    // Initialize the solenoid directly with the CTRE PCM
    sEAExtend = new Solenoid(1,PneumaticsModuleType.CTREPCM, Constants.robotConstants.kPneuExtendID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setExtend(){
    sEAExtend.set(true);
  }  

  public void setRetract(){
    sEAExtend.set(false);
  }  
}
