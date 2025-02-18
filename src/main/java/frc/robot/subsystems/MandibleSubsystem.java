// MANDIBLE IS THE CLIMBER

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class MandibleSubsystem extends SubsystemBase {
    private final SparkMax mandibleMotor;
    private final RelativeEncoder mandibleEncoder;

    private static final double CW_SPEED = -1;   // Adjust as needed
    private static final double CCW_SPEED = 1; // Adjust as needed
    private static final double STOP_SPEED = 0.0;

    


    public MandibleSubsystem(int motorPort) {
        mandibleMotor = new SparkMax(13, MotorType.kBrushless);
        mandibleEncoder = mandibleMotor.getEncoder();
        mandibleMotor.set(0);

    }

    public void moveClockwise() {
        mandibleMotor.set(CW_SPEED);
    }

    public void moveCounterClockwise() {
        mandibleMotor.set(CCW_SPEED);
    }

    public void stop() {
        mandibleMotor.set(STOP_SPEED);
    }

    public double getPosition() {
        return mandibleEncoder.getPosition(); // Returns encoder position
    }

    public void resetEncoder() {
        mandibleEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        // You can log encoder values for debugging
        // System.out.println("Climber Position: " + getPosition());
    }
}
