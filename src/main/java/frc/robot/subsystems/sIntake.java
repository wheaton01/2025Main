package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.intakeConstants;
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
public class sIntake extends SubsystemBase {
  private final AnalogInput aIntakeSensor;
  private final VictorSPX mIntake;
  private final Timer intakeTimer = new Timer();
  private boolean extraIntakeActive = false;
  private boolean coralPlaceMode = false;
  private boolean bHasCoral = false;
  private boolean bManual;
  private double stateManager =1;
  /** Creates a new sIntake. */
  public sIntake() {
      mIntake = new VictorSPX(robotConstants.kintakeSparkID);
      aIntakeSensor = new AnalogInput(robotConstants.kIntakeSensorID);
  }
/*state manager 1 = intake mode
state manager 2 = feed mode
state manager 3 = zero mode 
*/
  @Override
  public void periodic() {
    boolean coralDetected = getCoralSensor();

    SmartDashboard.putNumber("INTAKE STATE MANAGER", stateManager);
    SmartDashboard.putBoolean("CORAL DETECTED", coralDetected);
    SmartDashboard.putNumber("Inake Sensor Val", aIntakeSensor.getValue());
    if (stateManager == 1) {
      
      setMotorSpeed(intakeConstants.kIdleIntakeSpeed);
    } else if (stateManager == 2) {
      if (bManual) {
        setMotorSpeed(intakeConstants.kPlaceSpeed*.25);
      } else {
        setMotorSpeed(intakeConstants.kPlaceSpeed);
        bHasCoral = false;
      }
    } else if (stateManager == 3) {
      setMotorSpeed(.0);
    }else if (stateManager == 4) {
      if (bManual) {
        setMotorSpeed(intakeConstants.kBallIntakeSpeed*.25);
      } else {
        setMotorSpeed(intakeConstants.kBallIntakeSpeed);
      }
    }
    if (coralDetected 
        && stateManager != 2 
        && stateManager != 4
        && !bManual) //prevents interruption of algae intake
        {
      stateManager = 3;
      bHasCoral = true;
    }
    if(!bHasCoral && stateManager == 3){
      stateManager = 1;
    }
  



    // Update coral detection for the next loop
  }

  /**
   * Sets the intake to a given speed.
   * @param speed The speed (between -1.0 and 1.0)
   */
  public void setIntake(double speed) {
    
    mIntake.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /** Stops the intake motor. */
  public void setZero() {
    stateManager = 3;
    bManual = false;
  }
  public void setMotorSpeed(double speed){
    mIntake.set(VictorSPXControlMode.PercentOutput, speed);
  }

  /** Sets intake to idle speed. */
  public void setIntakeMode() {
    stateManager = 1;
    bManual = false;
  }
  public void hardResetIntake(){
  stateManager = 1;
  bHasCoral = false;
  bManual = false;
  }

  /** Runs intake at feeding speed. */
  public void setFeedIntake() {
  stateManager = 2;
  bManual = false;

  }
  /** Runs intake at ball intake speed. */
  public void setBallIntake() {
    stateManager = 4 ;
    bManual = false;

  }
  public void setManFeed(){
    stateManager = 2;
    bManual = true;
  }
  public void setManReverse(){
    stateManager = 4;
    bManual = true;
  }

  /**
   * Checks if the intake is in idle mode.
   * @return true if intake is running at idle speed.
   */
  public boolean getIntakeMode() {
    return mIntake.getMotorOutputPercent() == intakeConstants.kIdleIntakeSpeed;
  }

  /**
   * Checks if the coral sensor detects an object.
   * @return true if the sensor value is below the threshold (coral is present).
   */
  public boolean getCoralSensor() {
    return aIntakeSensor.getValue() > robotConstants.kintakeSensorThreshold;
  }

}
