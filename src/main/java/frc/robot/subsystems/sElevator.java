package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
    
    private static final int CURRENT_SPIKE_FILTER_CYCLES = 5; // Adjust as needed
    private int motor1CurrentSpikeCount = 0;
    private int motor2CurrentSpikeCount = 0;
    private PIDController mElevatorUpPid;
    private PIDController mElevatorUpPid2;
    private PIDController mElevatorDownPid;
    private SparkMax mElevator1, mElevator2;
    //private Encoder mElevatorEncoder;
    @SuppressWarnings("unused")
    private boolean tuningMode = false;
    private boolean bDownFlag = false;
    private boolean homingMode = false;
    private boolean motor1Homed = false;
    private boolean motor2Homed = false;
    private boolean startedHoming = false;
    double mELevatorSetpoint;
    RelativeEncoder mElevatorEncoder;
    RelativeEncoder mElevatorEncoder2;
    double position2;
    boolean atHome;
    DigitalInput homingSwitch;
    // Constructor
    public sElevator() {
        mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, SparkMax.MotorType.kBrushless);
        mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, SparkMax.MotorType.kBrushless);
        mElevatorEncoder = mElevator1.getEncoder();
        mElevatorEncoder2 = mElevator2.getEncoder();  
        mElevatorEncoder.setPosition(0);
        mElevatorEncoder2.setPosition(0);

        
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
        SmartDashboard.putNumber("elevator Pose",getHeight());
        SmartDashboard.putNumber("elevator Pose2",getHeight2());

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
            // ╔════════════════════════════╗
            // ║     Elevator Control       ║
            // ╚════════════════════════════╝
            if (!bDownFlag) {
                // ─── Ascending ───
                output = mElevatorUpPid.calculate(position) + elevatorConstants.kFeedForward;
            } else {
                // ─── Descending ───
                output = elevatorConstants.kdownSpeed + elevatorConstants.kFeedForward;
    
                // Stop the descent when near the setpoint
                boolean atSetpoint = Math.abs(mElevatorUpPid.getSetpoint() - position) < elevatorConstants.kPIDThreshold
                                     || (position - mElevatorUpPid.getSetpoint()) < 0;
    
                 if (atSetpoint) { 
                    if (!homingMode){
                    output = elevatorConstants.kFeedForward; // Hold position
                    bDownFlag = false; // Reset flag when target is reached
                    }
                    if(homingMode){
                        if (!getHomeSW()) {
                            output = elevatorConstants.kdownSpeed + elevatorConstants.kFeedForward;
                        }
                        if (getHomeSW()){
                            output = elevatorConstants.kFeedForward;
                            bDownFlag = false;
                        }
                    }
                }

                }
  
                SmartDashboard.putNumber("elevator Pose",mElevatorEncoder.getPosition());
                SmartDashboard.putBoolean("Homing mode",homingMode);
                SmartDashboard.putNumber("motorCurrent1", mElevator1.getOutputCurrent());
                SmartDashboard.putNumber("motorCurrent2", mElevator2.getOutputCurrent());

            // ╔════════════════════════════╗
            // ║   Apply Motor Output       ║
            // ╚════════════════════════════╝
            if (!startedHoming) {
            mElevator1.set(output);
            mElevator2.set(output);
            }
            if(homingMode&&!bDownFlag){
                homingMode();
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
                homingMode = false;
                bDownFlag = false;
                startedHoming = false;
            }else {
                if (height ==0){
                    homingMode = true;
                    motor1Homed = false;
                    motor2Homed = false;
                }else{
                    homingMode= false;
                }
                bDownFlag = true;
            }
            mElevatorUpPid.setSetpoint(height);
            // mElevatorDownPid.setSetpoint(height);
        } else {
            SmartDashboard.putString(getSubsystem(), "Requested Height > MAX HEIGHT");
        }

     }
    }

    // public void setElevatorOffset(double offset){
    //     if (elevatorConstants.kHomePose-offset>0 && elevatorConstants.kMaxHeight+offset<elevatorConstants.kMaxHeight){
    //     mElevatorUpPid.setSetpoint(offset+mELevatorSetpoint);
    //     }
 
    // }
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
        return mElevatorEncoder.getPosition()*elevatorConstants.kElevatorConversionFac+getoperatorOffset();
    }
    public double getHeight2() {
        return mElevatorEncoder2.getPosition()*elevatorConstants.kElevatorConversionFac+getoperatorOffset();
    }

    // Toggle tuning mode on/off
    public void setTuningMode(boolean isEnabled) {
        tuningMode = isEnabled;
    }
    private void homingMode() {
        startedHoming = true;
        if (!motor1Homed) {
            mElevator1.set(0.1);
            if (mElevator1.getOutputCurrent() > elevatorConstants.kCURRENT_THRESHOLD) {
                motor1CurrentSpikeCount++;
                if (motor1CurrentSpikeCount >= CURRENT_SPIKE_FILTER_CYCLES) {
                    mElevator1.set(0);
                    mElevatorEncoder.setPosition(0);
                    motor1Homed = true;
                }
            } else {
                motor1CurrentSpikeCount = 0; // Reset if current drops below threshold
            }
        }
    
        if (!motor2Homed) {
            mElevator2.set(0.1);
            if (mElevator2.getOutputCurrent() > elevatorConstants.kCURRENT_THRESHOLD) {
                motor2CurrentSpikeCount++;
                if (motor2CurrentSpikeCount >= CURRENT_SPIKE_FILTER_CYCLES) {
                    mElevator2.set(0);
                    mElevatorEncoder2.setPosition(0);
                    motor2Homed = true;
                }
            } else {
                motor2CurrentSpikeCount = 0; // Reset if current drops below threshold
            }
        }
    
        if (motor1Homed && motor2Homed) {
            startedHoming =false;
            homingMode = false;
            bDownFlag = false;
        }
    }
    double operatorOffset;
    public double getoperatorOffset()
    {
        return operatorOffset;
    }
    public void setoperatorOffset(double heightOffset)
    {
        if (Math.abs(heightOffset)>.1){
            operatorOffset = heightOffset*70;
        }else{operatorOffset = 0;}
        
    }
    public boolean getHomeSW(){
        return homingSwitch.get();
    }
    }



