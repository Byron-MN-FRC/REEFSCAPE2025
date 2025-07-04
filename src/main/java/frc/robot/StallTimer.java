package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StallTimer extends SubsystemBase{

    private TalonFX m_motor;
    private Timer m_stallTimer = new Timer();
    private double m_stallTime = 0.75; // Stall time in seconds

    public StallTimer(TalonFX motor) {
        this.m_motor = motor;
    }

    public StallTimer(TalonFX motor, double stallTime) {
        this.m_motor = motor;
        this.m_stallTime = stallTime;
    }

    @Override
    public void periodic() {
        boolean stallingCurrent = m_motor.getFault_StatorCurrLimit().getValue();
        if (m_stallTimer.isRunning()) {
            if (!stallingCurrent) {
                m_stallTimer.stop();
                m_stallTimer.reset();
            }
        } else {
            if (stallingCurrent) {
                m_stallTimer.restart();
            }
        }

    }

    public boolean isStalled() {
        return m_stallTimer.get() > m_stallTime; // try && isRunning() to avoid false positives
    }

}