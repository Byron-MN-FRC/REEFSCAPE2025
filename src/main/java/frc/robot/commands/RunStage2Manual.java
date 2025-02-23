package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RunStage2Manual extends Command {
    private final Elevator m_elevator;
    private final XboxController control;
    
  /** Creates a new ClawDrop. */
  public RunStage2Manual(Elevator subsystem, XboxController control) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_elevator = subsystem;
    this.control = control;
    addRequirements(m_elevator);
  }

@Override
public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    m_elevator.stopBothMotors();
}

@Override
public void execute() {
    // TODO Auto-generated method stub
    m_elevator.runWithJoystick(control);    

}

  
}
