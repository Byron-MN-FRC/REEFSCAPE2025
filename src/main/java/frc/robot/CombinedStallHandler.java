package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CombinedStallHandler extends SubsystemBase {

    private TalonFX m_motor;
    private Timer m_stallTimer = new Timer();
    private double m_stallTime = 0.75; // Stall time in seconds
    private double stallingSpeed = 0.05; // Speed threshold for stalling

    public CombinedStallHandler(TalonFX motor) {
        this.m_motor = motor;
    }

    public CombinedStallHandler(TalonFX motor, double stallTime) {
        this.m_motor = motor;
        this.m_stallTime = stallTime;
    }

    public CombinedStallHandler(TalonFX motor, double stallTime, double stallingSpeed) {
        this.m_motor = motor;
        this.m_stallTime = stallTime;
        this.stallingSpeed = stallingSpeed;
    }

    @Override
    public void periodic() {
        boolean stallingCurrent = m_motor.getFault_StatorCurrLimit().getValue();
        boolean stallingMotion = Math.abs(m_motor.getVelocity().getValueAsDouble()) < stallingSpeed;

        if (m_stallTimer.isRunning()) {
            if (!stallingCurrent || !stallingMotion) {
                m_stallTimer.stop();
                m_stallTimer.reset();
            }
        } else {
            if (stallingCurrent && stallingMotion) {
                m_stallTimer.restart();
            }
        }
    }

    public boolean isStalled() {
        return m_stallTimer.get() > m_stallTime; // try && isRunning() to avoid false positives
    }
}