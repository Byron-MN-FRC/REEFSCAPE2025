package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;

public class VisionDefaultCommand extends Command {
    Vision m_vision;
    String cam;

    public VisionDefaultCommand (Vision vision, String cam) {
        m_vision = vision;
        this.cam = cam;
        addRequirements(vision);
    }

    @Override 
    public void execute() {
        if (!DriverStation.isAutonomousEnabled()) {
            m_vision.updatePoseEstimation(cam);
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}