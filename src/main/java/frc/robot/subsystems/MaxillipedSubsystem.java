package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MaxillipedSubsystem extends SubsystemBase {
    private final VictorSPX maxillipedMotor;
    private boolean isExtended = true; // Track solenoid state

    public MaxillipedSubsystem(int motorID) {
        maxillipedMotor = new VictorSPX(motorID);
    }

    /**
     * Toggles the solenoid state between retracted (1) and extended (0).
     */
    public void toggleMaxilliped() {
        isExtended = !isExtended;
        maxillipedMotor.set(ControlMode.PercentOutput, isExtended ? 1.0 : 0.0); // 1 power = Retracted
    }

    /**
     * Returns whether the solenoid is extended.
     */
    public boolean isExtended() {
        return isExtended;
    }

    /**
     * Stops the motor (optional).
     */
    public void stop() {
        maxillipedMotor.set(ControlMode.PercentOutput, 0.0); // 0 power = Extended
    }
}