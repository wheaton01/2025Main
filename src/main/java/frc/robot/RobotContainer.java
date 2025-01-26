// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.climberCommands.setClimber;
import frc.robot.commands.intakeCommands.setIntake;
import frc.robot.subsystems.sClimber;
import frc.robot.subsystems.sElevator;
import frc.robot.subsystems.sEndAffector;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  sEndAffector sEndAffector;
  sClimber sClimber;
  sElevator sElevator;

  setIntake defaultIntake;
  setClimber defaultClimber;
  
  

  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Initialize sEndAffector in the constructor

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initializing Subsystems
    sEndAffector = new sEndAffector();
    sClimber = new sClimber();
    sElevator = new sElevator();

    setupCommands();
    
        // Configure the trigger bindings
        configureBindings();
      }
    
    
      private void setupCommands() {
         defaultIntake = new setIntake(false, false, 0, sEndAffector);
         defaultClimber =new setClimber(sClimber, false, false);


         sEndAffector.setDefaultCommand(defaultIntake);
         sClimber.setDefaultCommand(defaultClimber);
         //sElevator.setDefaultCommand(defaultElevator);
      }
    
    
      private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }
  


  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //todo : this is all temporary
    return Autos.getAutonomousCommand(sEndAffector);
  }
}
