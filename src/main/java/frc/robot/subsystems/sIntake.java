package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.intakeConstants;

public class sIntake extends SubsystemBase {
  private final AnalogInput aIntakeSensor;
  private final VictorSPX mIntake;
  private final Timer intakeTimer = new Timer();
  private boolean coralDetectedLastCycle = false;
  private boolean extraIntakeActive = false;
  private boolean coralPlaceMode = false;
  /** Creates a new sIntake. */
  public sIntake() {
      mIntake = new VictorSPX(robotConstants.kintakeSparkID);
      aIntakeSensor = new AnalogInput(robotConstants.kIntakeSensorID);
  }

  @Override
  public void periodic() {
    boolean coralDetected = getCoralSensor();

    // Log sensor values for debugging
    SmartDashboard.putNumber("Intake Sensor Value", aIntakeSensor.getValue());
    SmartDashboard.putBoolean("Intake Mode", getIntakeMode());
    SmartDashboard.putBoolean("Coral Detected", coralDetected);
    SmartDashboard.putBoolean("Extra Intake Active", extraIntakeActive);
    SmartDashboard.putNumber("Intake Timer", intakeTimer.get());

    // If a new coral is detected, reset and start the timer
    if (!coralPlaceMode){
    if (coralDetected && !coralDetectedLastCycle) {
        intakeTimer.reset(); // Ensures a fresh start every time a new coral is detected
        intakeTimer.start();
        extraIntakeActive = false; // Normal intake process
    }

    // If the coral is no longer detected but was previously, start the extra intake phase
    if (!coralDetected && coralDetectedLastCycle) {
        intakeTimer.reset();
        intakeTimer.start();
        extraIntakeActive = true;
    }

    // Keep the intake running if coral is detected OR if the extra intake phase is still running
    if (coralDetected || (extraIntakeActive && intakeTimer.get() < intakeConstants.kIntakeTime)) {
        setIntakeMode();
    } else {
        setZero();
        if (extraIntakeActive && intakeTimer.get() >= 0.5) {
            intakeTimer.stop();
            extraIntakeActive = false;
        }
      }
    }
    if (coralPlaceMode){
      mIntake.set(VictorSPXControlMode.Velocity, 1.0);
    }

    // Update coral detection for the next loop
    coralDetectedLastCycle = coralDetected;
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
    mIntake.set(VictorSPXControlMode.PercentOutput, 0);
  }

  /** Sets intake to idle speed. */
  public void setIntakeMode() {
    mIntake.set(VictorSPXControlMode.PercentOutput, intakeConstants.kIdleIntakeSpeed);
    coralPlaceMode = false;

  }

  /** Runs intake at feeding speed. */
  public void setFeedIntake() {
    mIntake.set(VictorSPXControlMode.PercentOutput, intakeConstants.kIntakeSpeed);
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
    return aIntakeSensor.getValue() < robotConstants.kintakeSensorThreshold;
  }
  public void setCoralPlaceMode(){
    coralPlaceMode = true;
  }
}
