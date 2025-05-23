// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: SequentialCommandGroup.

package frc.robot.commands.AutonomousCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.PoseSetter;
import frc.robot.Robot;
import frc.robot.commands.CoralClawIntake;
import frc.robot.commands.MoveElevator;
import frc.robot.commands.MoveShoulder;
import frc.robot.subsystems.Coral;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shoulder;
/**
 *
 */
public class AutonGrabCoral extends SequentialCommandGroup {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

   // public AutoRaiseCoral(Elevator elevator){

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    public AutonGrabCoral(Shoulder m_shoulder, Elevator m_elevator, Coral m_claw){
        // Add Commands here:
        // Also add parallel commands using the
        //
        addCommands(
            Commands.parallel(
            new InstantCommand(() -> Robot.getInstance().goalArrangementOthers(PoseSetter.Feeder)),
              new MoveShoulder(m_shoulder),
              new MoveElevator(m_elevator)),
              new CoralClawIntake(m_claw)
        );
        
    }

    @Override
    public boolean runsWhenDisabled() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
        return false;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DISABLED
    }
}
