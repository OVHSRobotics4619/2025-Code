// CRUSHER CLAW IS THE GRABBER

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CrusherClawSubsystem extends SubsystemBase {
    private final SparkMax crusherClawMotor;
    private final RelativeEncoder crusherClawEncoder;

    private static final double UP_SPEED = -0.1; // Adjust as needed
    private static final double DOWN_SPEED = 0.1; // Adjust as needed
    private static final double HOLDING_POWER = -0.02; // Power needed to prevent arm from falling naturally, but not enough to move the arm
    private static final double STOP_SPEED = 0.0; // Stop

    public static final double ARM_UP_POSITION = 3.0; // Set actual encoder value
    public static final double ARM_DOWN_POSITION = 40.0; // Set actual encoder value



    public CrusherClawSubsystem(int motorPort) {
        crusherClawMotor = new SparkMax(12, MotorType.kBrushless);
        crusherClawEncoder = crusherClawMotor.getEncoder();
        crusherClawMotor.set(0);

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop
                .p(0.1)
                .i(0.0)
                .d(0.1)
                .outputRange(-0.3, 0.3);

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

    public void moveToPosition(double targetPosition) {
        double error = targetPosition - crusherClawEncoder.getPosition();
        double output =  0.1 * error; // kP is a tuned PID constant
        crusherClawMotor.set(output);
    }

    public boolean isAtPosition(double targetPosition) {
        return Math.abs(crusherClawEncoder.getPosition() - targetPosition) < 5.0; // Tolerance
    }

    @Override
    public void periodic() {
        // Log encoder values for debugging
        // System.out.println("Elevator Position: " + getPosition());
    }
}
