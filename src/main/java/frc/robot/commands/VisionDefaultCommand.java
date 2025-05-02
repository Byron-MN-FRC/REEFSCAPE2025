public class VisionDefaultCommand extends Command {
    Vision m_vision;

    public VisionDefaultCommand (Vision vision, String cam) {
        m_vision = vision;
        addRequirements(vision);
    }

    @Override 
    public void execute() {
        if (!DriverStation.isAutonomousEnabled) {
            m_vision.updatePoseEstimation(cam);
        }
    }

    @Override
    public void runsWhenDisabled() {
        true;
    }
}