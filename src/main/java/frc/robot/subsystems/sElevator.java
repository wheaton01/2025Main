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
                                             robotConstants.elevatorConstants.kI, 
                                             robotConstants.elevatorConstants.kD);
        mElevatorDownPid.setIZone(elevatorConstants.kIZone);
        mElevatorDownPid.setTolerance(elevatorConstants.kTolerance);
    }

    @Override
    public void periodic() {
        double position = getHeight();
        SmartDashboard.putNumber("Elevator Position", position);
        SmartDashboard.putNumber("Elevator Setpoint", mElevatorUpPid.getSetpoint());  // or mElevatorDownPid.getSetpoint() if down

        // Select the PID controller based on direction of movement
        double PIDOutput = 0;
        if (mElevatorUpPid.getSetpoint() > position) {
            // Moving up
            PIDOutput = mElevatorUpPid.calculate(position);
        } else {
            // Moving down
            PIDOutput = mElevatorDownPid.calculate(position);
        }

        // Apply the PID output with feedforward
        double finalOutput = PIDOutput + elevatorConstants.kFeedForward;
        mElevator1.set(finalOutput);
        // mElevator2.set(-finalOutput);  // Optionally mirror the second motor
    }

    // Set the desired height (pose) for the elevator
    public void setElevatorPose(double height) {
        if (height < elevatorConstants.kMaxHeight) {
            // Store the setpoint in both up/down PID controllers
            mElevatorUpPid.setSetpoint(height);
            mElevatorDownPid.setSetpoint(height);
        } else {
            SmartDashboard.putString(getSubsystem(), "Requested Height > MAX HEIGHT");
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


