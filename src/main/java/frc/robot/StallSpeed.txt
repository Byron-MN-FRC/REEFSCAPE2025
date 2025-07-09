package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

public class StallSpeed {

    public static boolean checkStalled(TalonFX m_motor) {
        boolean stallingCurrent = m_motor.getFault_StatorCurrLimit().getValue();
        boolean stallingMotion = Math.abs(m_motor.getVelocity().getValueAsDouble()) < 0.05;
        if (stallingCurrent && stallingMotion) {
            return true;
        } else {
            return false;
        }
    }
}