package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.commands.SwerveCommands.DriveToPoseCommand;
import frc.robot.subsystems.sClimber;
import frc.robot.subsystems.sElevator;
import frc.robot.subsystems.sEndAffector;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    sEndAffector sEndAffector;
    sClimber sClimber;
    sElevator sElevator;
    SwerveSubsystem swerveSubsystem;

    setIntake defaultIntake;
    setClimber defaultClimber;

    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    public final CommandXboxController m_operatorController =
        new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    public RobotContainer() {
        sEndAffector = new sEndAffector();
        sClimber = new sClimber();
        sElevator = new sElevator();
        swerveSubsystem = new SwerveSubsystem(new File("path/to/config"));

        setupCommands();
        configureBindings();
    }

    private void setupCommands() {
        defaultIntake = new setIntake(false, false, 0, sEndAffector);
        defaultClimber = new setClimber(sClimber, false, false);

        sEndAffector.setDefaultCommand(defaultIntake);
        sClimber.setDefaultCommand(defaultClimber);
    }

    private void configureBindings() {
        // Bind left stick to standard swerve drive
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
                m_driverController::getRightTriggerAxis
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
                m_driverController::getRightTriggerAxis
            ));
    }

    public Command getAutonomousCommand() {
        return Autos.getAutonomousCommand(sEndAffector);
    }
}