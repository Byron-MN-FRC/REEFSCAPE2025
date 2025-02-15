// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Command.

package frc.robot.commands;
import javax.swing.plaf.InsetsUIResource;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Wrist;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class GrabCoralHigh extends SequentialCommandGroup {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

   // public AutoRaiseCoral(Elevator elevator){

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    public GrabCoralHigh(Shoulder m_shoulder, Elevator m_elevator, Wrist m_wrist, Claw m_claw){
        // Add Commands here:
        // Also add parallel commands using the
        //
        addCommands(
              new MoveWrist(m_wrist),           
              new MoveShoulder(m_shoulder),
              new MoveElevator(m_elevator),
              new ClawIntake(m_claw),
              new InstantCommand(() -> Robot.getInstance().currentArrangementOthers(PoseSetter.Feeder))
                  
        );

    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
