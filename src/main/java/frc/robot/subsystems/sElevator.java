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

    private PIDController mElevatorPid;
    private SparkMax mElevator1, mElevator2;
    double nTuningFF = 0.0; // Tuning feedforward value
    // Tuning mode boolean
    private boolean tuningMode = true; //TURN OFF ONCE TUNED
     Encoder mElevatorEncoder;

    // Constructor
    public sElevator() {
        mElevator1 = new SparkMax(robotConstants.kelevatorSparkID1, SparkMax.MotorType.kBrushless);
        mElevator2 = new SparkMax(robotConstants.kelevatorSparkID2, SparkMax.MotorType.kBrushless);
        //encoder init

        mElevatorEncoder = new Encoder(elevatorConstants.kEncoderA, elevatorConstants.kEncoderB);
        mElevatorEncoder.setSamplesToAverage(5);
        mElevatorEncoder.setReverseDirection(true);
        mElevatorEncoder.setDistancePerPulse(elevatorConstants.kElevatorConversionFac);

        // Initialize the PID controller with default values
        mElevatorPid = new PIDController(robotConstants.elevatorConstants.kP, 
                                         robotConstants.elevatorConstants.kI, 
                                         robotConstants.elevatorConstants.kD);
                    

        mElevatorPid.setTolerance(elevatorConstants.kTolerance);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Current Elevator Pose",mElevatorEncoder.getDistance());
        SmartDashboard.putNumber("Current Elevator Velocity",mElevator1.get());
        // Tuning Mode: Check if we are in tuning mode
        if (tuningMode) {
            // Update PID constants from SmartDashboard if tuning mode is on
            mElevatorPid.setP(SmartDashboard.getNumber("PID P", robotConstants.elevatorConstants.kP));
            mElevatorPid.setI(SmartDashboard.getNumber("PID I", robotConstants.elevatorConstants.kI));
            mElevatorPid.setD(SmartDashboard.getNumber("PID D", robotConstants.elevatorConstants.kD));
            nTuningFF = SmartDashboard.getNumber("Tuning Feedforward", robotConstants.elevatorConstants.kFeedForward);
            double setpoint = SmartDashboard.getNumber("Elevator Setpoint Input", mElevatorPid.getSetpoint());
            mElevatorPid.setSetpoint(setpoint);

        }

        // Get the elevator position from the encoder
        double position = getHeight();

        // Update SmartDashboard with elevator position and setpoint
        SmartDashboard.putNumber("Elevator Position", position);
        SmartDashboard.putNumber("Elevator Setpoint", mElevatorPid.getSetpoint());

        // Calculate PID output based on current position
        double PIDOutput = mElevatorPid.calculate(position + robotConstants.elevatorConstants.kFeedForward+ nTuningFF);

        // Control the motors based on the PID output
        mElevator1.set(PIDOutput);
        // mElevator2.set(-PIDOutput);  // Opposing motor for synchronization
    }

    // Set the desired height (pose) for the elevator
    public void setElevatorPose(double height) {
        if(height<elevatorConstants.kMaxHeight){
        mElevatorPid.setSetpoint(height);
        }else {SmartDashboard.putString(getSubsystem(),"requested Height > MAX HEIGHT");}
    }

    // Check if the elevator is at the setpoint
    public boolean isAtSetpoint() {
        return mElevatorPid.atSetpoint();
    }
    public void zeroElevator(){

    }

    // Method to toggle tuning mode on/off
    public void setTuningMode(boolean isEnabled) {
        tuningMode = isEnabled;
    }
    public void setManualMotors(double speed) {
        mElevator1.set(speed+elevatorConstants.kFeedForward);
    }
    public double getHeight(){
        return mElevatorEncoder.getDistance();
    } 
}
