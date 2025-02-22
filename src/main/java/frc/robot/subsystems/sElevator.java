package frc.robot.subsystems;

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
    private PIDController mElevatorUpPid2;
    private PIDController mElevatorDownPid;
    private SparkMax mElevator1, mElevator2;
    //private Encoder mElevatorEncoder;
    @SuppressWarnings("unused")
    private boolean tuningMode = false;
    private boolean bDownFlag = false;
    double mELevatorSetpoint;
    RelativeEncoder mElevatorEncoder;
    RelativeEncoder mElevatorEncoder2;
    double position2;
    // Constructor
    public sElevator() {
        mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, SparkMax.MotorType.kBrushless);
        mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, SparkMax.MotorType.kBrushless);
        mElevatorEncoder = mElevator1.getEncoder();
        mElevatorEncoder2 = mElevator2.getEncoder();

        
        // mElevatorEncoder = new Encoder(elevatorConstants.kEncoderA, elevatorConstants.kEncoderB);
        // mElevatorEncoder.setSamplesToAverage(5);
        // mElevatorEncoder.setReverseDirection(true);
        // mElevatorEncoder.setDistancePerPulse(elevatorConstants.kElevatorConversionFac);

        // Initialize separate PID controllers for upward and downward motion
        mElevatorUpPid = new PIDController(robotConstants.elevatorConstants.kP_up, 
                                           robotConstants.elevatorConstants.kI, 
                                           robotConstants.elevatorConstants.kD);
        mElevatorUpPid2 = new PIDController(robotConstants.elevatorConstants.kP_up, 
                                           robotConstants.elevatorConstants.kI, 
                                           robotConstants.elevatorConstants.kD);
        mElevatorUpPid.setIZone(elevatorConstants.kIZone);
        mElevatorUpPid.setTolerance(elevatorConstants.kTolerance);
        
        mElevatorUpPid2.setIZone(elevatorConstants.kIZone);
        mElevatorUpPid2.setTolerance(elevatorConstants.kTolerance);
                                   
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
            SmartDashboard.putBoolean("Down Flag ", bDownFlag);
    
            double position = getHeight(); // Current elevator height
            if(elevatorConstants.btwoMotorMode){
                SmartDashboard.putNumber("Elevator Position 2 ", getHeight2());
                position2 = getHeight2(); // Current elevator height

            }
            SmartDashboard.putNumber("Elevator Position 1 ", position);
            SmartDashboard.putNumber("Elevator Setpoint 1 ", mElevatorUpPid.getSetpoint());
    
            double output = 0.0; // Motor power output
            double output2 = 0.0;
    
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
            if (!bDownFlag && elevatorConstants.btwoMotorMode) {
                // ─── Ascending ───
                output2 = mElevatorUpPid2.calculate(position2) + elevatorConstants.kFeedForward;
            } else {
                // ─── Descending ───
                output2 = elevatorConstants.kdownSpeed + elevatorConstants.kFeedForwardDown;
    
                // Stop the descent when near the setpoint
                boolean atSetpoint2 = Math.abs(mElevatorUpPid2.getSetpoint() - position2) < elevatorConstants.kPIDThreshold
                                     || (position2 - mElevatorUpPid2.getSetpoint()) < 0;
    
                if (atSetpoint2) { 
                    output2 = elevatorConstants.kFeedForward; // Hold position
                    bDownFlag = false; // Reset flag when target is reached
                }
            }
    
            // ╔════════════════════════════╗
            // ║   Apply Motor Output       ║
            // ╚════════════════════════════╝
            mElevator1.set(output);
            if(elevatorConstants.btwoMotorMode){
                mElevator2.set(output2);
            }
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
    //this looks way more complicated than it is. 
    //it just is a command to manually move the elevator and so we can run this as two separate motors instead of one being a follower
    public Command setElevator(DoubleSupplier speed) {
        if (elevatorConstants.btwoMotorMode) {
            return new RunCommand(() -> mElevator1.set(speed.getAsDouble()), this)
                .alongWith(new RunCommand(() -> mElevator2.set(speed.getAsDouble()), this));
        } else {
            return new RunCommand(() -> mElevator1.set(speed.getAsDouble()), this);
        }    
    }

    // Check if the elevator is at the setpoint
    public boolean isAtSetpoint() {
        return mElevatorUpPid.atSetpoint() || mElevatorDownPid.atSetpoint();
    }

    public double getHeight() {
        return mElevatorEncoder.getPosition()*elevatorConstants.kElevatorConversionFac;
    }
    public double getHeight2() {
        return mElevatorEncoder2.getPosition()*elevatorConstants.kElevatorConversionFac;
    }

    // Toggle tuning mode on/off
    public void setTuningMode(boolean isEnabled) {
        tuningMode = isEnabled;
    }
}



