package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import edu.wpi.first.math.controller.PIDController;
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

public class sElevator extends SubsystemBase {

    private PIDController mElevatorUpPid;
    private PIDController mElevatorDownPid;
    private SparkMax mElevator1, mElevator2;
    //private Encoder mElevatorEncoder;
    private boolean tuningMode = false;
    private boolean bDownFlag = false;
    double mELevatorSetpoint;
    RelativeEncoder mElevatorEncoder;
    // Constructor
    public sElevator() {
        mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, SparkMax.MotorType.kBrushless);
        mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, SparkMax.MotorType.kBrushless);
        mElevatorEncoder = mElevator1.getEncoder();
        
        // mElevatorEncoder = new Encoder(elevatorConstants.kEncoderA, elevatorConstants.kEncoderB);
        // mElevatorEncoder.setSamplesToAverage(5);
        // mElevatorEncoder.setReverseDirection(true);
        // mElevatorEncoder.setDistancePerPulse(elevatorConstants.kElevatorConversionFac);

        // Initialize separate PID controllers for upward and downward motion
        mElevatorUpPid = new PIDController(robotConstants.elevatorConstants.kP_up, 
                                           robotConstants.elevatorConstants.kI, 
                                           robotConstants.elevatorConstants.kD);
        mElevatorUpPid.setIZone(elevatorConstants.kIZone);
        mElevatorUpPid.setTolerance(elevatorConstants.kTolerance);

        mElevatorDownPid = new PIDController(robotConstants.elevatorConstants.kP_down, 
                                             robotConstants.elevatorConstants.kI_down, 
                                             robotConstants.elevatorConstants.kD_down);
        mElevatorDownPid.setIZone(elevatorConstants.kIZone);
        mElevatorDownPid.setTolerance(elevatorConstants.kTolerance);

    }
    @Override
    public void periodic() {
        // ╔════════════════════════════╗
        // ║       Debugging Output     ║
        // ╚════════════════════════════╝
        if (!elevatorConstants.btestMode) {
            SmartDashboard.putBoolean("Down Flag", bDownFlag);
    
            double position = getHeight(); // Current elevator height
            SmartDashboard.putNumber("Elevator Position", position);
            SmartDashboard.putNumber("Elevator Setpoint", mElevatorUpPid.getSetpoint());
    
            double output = 0.0; // Motor power output
    
            // ╔════════════════════════════╗
            // ║     Elevator Control       ║
            // ╚════════════════════════════╝
            if (!bDownFlag) {
                // ─── Ascending ───
                output = mElevatorUpPid.calculate(position) + elevatorConstants.kFeedForward;
            } else {
                // ─── Descending ───
                output = elevatorConstants.kdownSpeed + elevatorConstants.kFeedForwardDown;
    
                // Stop the descent when near the setpoint
                boolean atSetpoint = Math.abs(mElevatorUpPid.getSetpoint() - position) < elevatorConstants.kPIDThreshold
                                     || (position - mElevatorUpPid.getSetpoint()) < 0;
    
                if (atSetpoint) { 
                    output = elevatorConstants.kFeedForward; // Hold position
                    bDownFlag = false; // Reset flag when target is reached
                }
            }
    
            // ╔════════════════════════════╗
            // ║   Apply Motor Output       ║
            // ╚════════════════════════════╝
            mElevator1.set(output);
        }
    }
    
    
    
    // Set the desired height (pose) for the elevator
    public void setElevatorPose(double height) {
        mELevatorSetpoint = height;
        if (height < elevatorConstants.kMaxHeight) {
            if (height != mElevatorUpPid.getSetpoint()){
            // Store the setpoint in both up/down PID controllers
            if (height > getHeight()) {
                bDownFlag = false;
            }else {
                bDownFlag = true;
            }
            mElevatorUpPid.setSetpoint(height);
            // mElevatorDownPid.setSetpoint(height);
        } else {
            SmartDashboard.putString(getSubsystem(), "Requested Height > MAX HEIGHT");
        }
     }
    }

    public void setElevatorOffset(double offset){
        if (elevatorConstants.kHomePose-offset>0 && elevatorConstants.kMaxHeight+offset<elevatorConstants.kMaxHeight){
        mElevatorUpPid.setSetpoint(offset+mELevatorSetpoint);
        }
 
    }
    DoubleSupplier speed;
    public Command setElevator(DoubleSupplier speed) {
        return new RunCommand(() -> mElevator1.set(speed.getAsDouble()), this);
    }

    // Check if the elevator is at the setpoint
    public boolean isAtSetpoint() {
        return mElevatorUpPid.atSetpoint() || mElevatorDownPid.atSetpoint();
    }

    public double getHeight() {
        return mElevatorEncoder.getPosition()*elevatorConstants.kElevatorConversionFac;
    }

    // Toggle tuning mode on/off
    public void setTuningMode(boolean isEnabled) {
        tuningMode = isEnabled;
    }
}


