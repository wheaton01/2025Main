package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.robotConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.*;
import edu.wpi.first.math.controller.PIDController;

public class sElevator extends SubsystemBase {

    private PIDController mElevatorUpPid;
    private PIDController mElevatorDownPid;
    private SparkMax mElevator1, mElevator2;
    private Encoder mElevatorEncoder;
    private boolean tuningMode = false;
    private boolean bDownFlag = false;

    // Constructor
    public sElevator() {
        mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, SparkMax.MotorType.kBrushless);
        mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, SparkMax.MotorType.kBrushless);

        mElevatorEncoder = new Encoder(elevatorConstants.kEncoderA, elevatorConstants.kEncoderB);
        mElevatorEncoder.setSamplesToAverage(5);
        mElevatorEncoder.setReverseDirection(true);
        mElevatorEncoder.setDistancePerPulse(elevatorConstants.kElevatorConversionFac);

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
        SmartDashboard.putBoolean("downflag", bDownFlag);

        double position = getHeight();
        SmartDashboard.putNumber("Elevator Position", position);
        SmartDashboard.putNumber("Elevator Setpoint", mElevatorUpPid.getSetpoint());  // or mElevatorDownPid.getSetpoint() if down
    
        // Select the PID controller based on direction of movement
        double PIDOutput = 0;
    
        if (!bDownFlag) {
            // Moving up with PID
            PIDOutput = mElevatorUpPid.calculate(position);
            mElevator1.set(PIDOutput + elevatorConstants.kFeedForward);
        } else {
            // Moving down with linear control until close enough to the setpoint
            double downSpeed = elevatorConstants.kdownSpeed;  // Adjust this value as needed for smoothness
            mElevator1.set(downSpeed + elevatorConstants.kFeedForwardDown);  // Add feedforward for gravity compensation
             if (Math.abs(mElevatorUpPid.getSetpoint()-getHeight())<elevatorConstants.kPIDThreshold){ {
                    mElevator1.set(elevatorConstants.kFeedForward);  // Stop once it's at the setpoint
                    bDownFlag = false;
                
            }
        }
    }
    
    
    // Set the desired height (pose) for the elevator
    public void setElevatorPose(double height) {
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

    // Check if the elevator is at the setpoint
    public boolean isAtSetpoint() {
        return mElevatorUpPid.atSetpoint() || mElevatorDownPid.atSetpoint();
    }

    public double getHeight() {
        return mElevatorEncoder.getDistance();
    }

    // Toggle tuning mode on/off
    public void setTuningMode(boolean isEnabled) {
        tuningMode = isEnabled;
    }
}


