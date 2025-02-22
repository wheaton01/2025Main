package frc.robot.commands.elevatorCommands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sElevator;
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
public class setElevatorOffset extends Command {
    private final sElevator m_elevator;
    private final DoubleSupplier offsetSupplier;
    private double offset;
    public setElevatorOffset(sElevator elevator, DoubleSupplier offsetSupplier) {
        this.m_elevator = elevator;
        this.offsetSupplier = offsetSupplier;
        addRequirements(elevator); // Ensures no other command interrupts
    }

    @Override
    public void execute() {
        offset =  offsetSupplier.getAsDouble();
        m_elevator.setElevatorOffset(100*offset);
        //double speed = m_speedSupplier.getAsDouble(); // Get joystick value
        //m_elevator.setManualMotors(speed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}
