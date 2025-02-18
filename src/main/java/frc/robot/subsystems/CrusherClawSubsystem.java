// CRUSHER CLAW IS THE GRABBER

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CrusherClawSubsystem extends SubsystemBase {
    private final SparkMax crusherClawMotor;
    private final RelativeEncoder crusherClawEncoder;

    private static final double UP_SPEED = -0.1;   // Adjust as needed
    private static final double DOWN_SPEED = 0.1; // Adjust as needed
    private static final double HOLDING_POWER = -0.03;
    private static final double STOP_SPEED = 0.0;

    


    public CrusherClawSubsystem(int motorPort) {
        crusherClawMotor = new SparkMax(12, MotorType.kBrushless);
        crusherClawEncoder = crusherClawMotor.getEncoder();
        crusherClawMotor.set(0);

    }

    public void moveUp() {
        crusherClawMotor.set(UP_SPEED);
    }

    public void moveDown() {
        crusherClawMotor.set(DOWN_SPEED);
    }

    public void stop() {
        crusherClawMotor.set(HOLDING_POWER);
    }

    public double getPosition() {
        return crusherClawEncoder.getPosition(); // Returns encoder position
    }

    public void resetEncoder() {
        crusherClawEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // You can log encoder values for debugging
        // System.out.println("Elevator Position: " + getPosition());
    }
}
