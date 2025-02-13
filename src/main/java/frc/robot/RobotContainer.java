package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.robotConstants.elevatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.setCHaptics;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.elevatorCommands.setElevatorPose;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.sClimber;
import frc.robot.subsystems.sControllerHaptics;
import frc.robot.subsystems.sElevator;
import frc.robot.subsystems.sEndAffector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    sEndAffector sEndAffector;
    sClimber sClimber;
    sElevator sElevator;
    SwerveSubsystem swerveSubsystem;

    setIntake defaultIntake, intakeBall;
    setClimber defaultClimber;
    sControllerHaptics m_controllerHaptics;
    setElevatorPose setL4Pose, setL3Pose,setL2Pose,setL1Pose, setHomePose;


    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);


    public RobotContainer() {
        sEndAffector = new sEndAffector();
        sClimber = new sClimber();
        sElevator = new sElevator();
        m_controllerHaptics = new sControllerHaptics(m_driverController, m_operatorController);

        swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
        "swerve/neo"));
        setupCommands();
        configureBindings();
    }

    private void setupCommands() {
        //Intake Commands
        defaultIntake = new setIntake(false, false, 0,0, sEndAffector);
        defaultClimber = new setClimber(sClimber, false, false);
        intakeBall = new setIntake(false, true, 0.0,-1.0, sEndAffector);

        //Elevator Commands
        setL4Pose = new   setElevatorPose(sElevator, elevatorConstants.kL4Height);
        setL3Pose = new   setElevatorPose(sElevator, elevatorConstants.kL3Height);
        setL2Pose = new   setElevatorPose(sElevator,elevatorConstants.kL2Height);
        setL1Pose = new   setElevatorPose(sElevator,elevatorConstants.kL1Height);
        setHomePose = new setElevatorPose(sElevator,elevatorConstants.kHomePose);


        //Setting Default Commands
        sEndAffector.setDefaultCommand(defaultIntake);
        sClimber.setDefaultCommand(defaultClimber);
    }

    private void configureBindings() {
        driverControls();
        operatorControls();
    }
    public void driverControls(){
        swerveSubsystem.setDefaultCommand(
            swerveSubsystem.driveCommand(
                m_driverController::getLeftX,
                m_driverController::getLeftY,
                m_driverController::getRightX
            )
        );

        // Bind right trigger to drive to nearest AprilTag pose to the right
        new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.5)
            .whileTrue(new DriveToPoseCommand(
                swerveSubsystem,
                swerveConstants.MAX_SPEED,
                m_driverController::getLeftX, // xOffset
                m_driverController::getLeftY, // yOffset
                m_driverController::getRightX, // rotationOffset
                m_driverController::getLeftTriggerAxis,
                m_driverController::getRightTriggerAxis,
                false,
                false
            ));

        // Bind left trigger to drive to nearest AprilTag pose to the left
        new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.5)
            .whileTrue(new DriveToPoseCommand(
                swerveSubsystem,
                swerveConstants.MAX_SPEED,
                m_driverController::getLeftX, // xOffset
                m_driverController::getLeftY, // yOffset
                m_driverController::getRightX, // rotationOffset
                m_driverController::getLeftTriggerAxis,
                m_driverController::getRightTriggerAxis,
                false,
                false
            ));
            // Bind B button to drive to nearest AprilTag pose at center
            new Trigger(m_driverController.b())
                        .whileTrue(new DriveToPoseCommand(
                            swerveSubsystem,
                            swerveConstants.MAX_SPEED,
                            m_driverController::getLeftX, // xOffset
                            m_driverController::getLeftY, // yOffset
                            m_driverController::getRightX, // rotationOffset
                            m_driverController::getLeftTriggerAxis,
                            m_driverController::getRightTriggerAxis,
                            false,
                            false // Set to false for non-autonomous mode, change as needed
                        ));
            new Trigger(m_driverController.rightBumper())
                        .whileTrue(new DriveToPoseCommand(
                            swerveSubsystem,
                            swerveConstants.MAX_SPEED,
                            m_driverController::getLeftX, // xOffset
                            m_driverController::getLeftY, // yOffset
                            m_driverController::getRightX, // rotationOffset
                            m_driverController::getLeftTriggerAxis,
                            m_driverController::getRightTriggerAxis,
                            true,
                            false // Set to false for non-autonomous mode, change as needed
                        ));
                        
      

    }

    public void operatorControls() {
        // ------------------------- Preset Pose Commands ------------------------- //
        new Trigger(m_operatorController.a()).onTrue(setL1Pose);
        new Trigger(m_operatorController.b()).onTrue(setL3Pose);
        new Trigger(m_operatorController.x()).onTrue(setL2Pose);
        new Trigger(m_operatorController.a()).onTrue(setHomePose); // Note: Duplicated A button binding?
    
        // ----------------------- Climber Safety Override ----------------------- //
        new Trigger(m_operatorController.rightStick()
            .and(m_operatorController.leftStick()))
            .onTrue(new ParallelCommandGroup(
                new setCHaptics(m_controllerHaptics, 0.8).withTimeout(1.2),
                new InstantCommand(sClimber::disableSafety)
            ));
    
        // --------------------------- Intake Controls --------------------------- //
    
        // Deploy intake when Left Bumper is pressed, retract when released
        new Trigger(m_operatorController.leftBumper().negate())
            .onTrue(new setIntake(false, true, 0, 0, sEndAffector))   // Deploy
            .onFalse(new setIntake(false, false, 0, 0, sEndAffector)); // Retract
    
        // Run intake when Left Trigger is pressed beyond 0.2 threshold
        new Trigger(m_operatorController.leftTrigger(0.2))
            .onTrue(new setIntake(false, true, 1.0, 0, sEndAffector))  // Intake game piece
            .onFalse(new setIntake(false, true, 0, 0, sEndAffector));  // Stop intake
    
        // Eject game piece when Right Trigger is pressed beyond 0.2 threshold
        new Trigger(m_operatorController.rightTrigger(0.2))
            .onTrue(new setIntake(false, true, -1.0, 0, sEndAffector)) // Reverse intake (Eject)
            .onFalse(new setIntake(false, true, 0, 0, sEndAffector));  // Stop intake
    
        // Enable Coral Sensor for automatic intake when Right Bumper is pressed
        new Trigger(m_operatorController.rightBumper())
            .onTrue(new setIntake(true, true, 1.0, 0, sEndAffector))  // Auto intake using Coral Sensor
            .onFalse(new setIntake(true, true, 0, 0, sEndAffector));  // Stop intake
    
        // Placing Mode (Y Button) - Runs the placing motor
        new Trigger(m_operatorController.y())
            .onTrue(new setIntake(false, true, 0, 1.0, sEndAffector)) // Start placing motor
            .onFalse(new setIntake(false, true, 0, 0, sEndAffector)); // Stop placing motor
    }
    
    

    public Command getAutonomousCommand() {
        return Autos.getAutonomousCommand(sEndAffector);
    }
}