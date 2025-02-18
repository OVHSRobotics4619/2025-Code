// OMMATOPHORE IS THE ELEVATOR

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class OmmatophoreSubsystem extends SubsystemBase {
    private final SparkMax ommatophoreMotor;
    private final RelativeEncoder ommatophoreEncoder;
    private final SparkClosedLoopController ommatophorePID;

    private static final double UP_SPEED = 0.2;   // Adjust as needed
    private static final double DOWN_SPEED = -0.2; // Adjust as needed
    private static final double HOLDING_POWER = 0.01;
    private static final double STOP_SPEED = 0.0;

    public static final double[] STAGES = {55.0, 106.0, 215.0}; // Stage positions
    private int currentStage = 0; // Start at bottom stage
    private boolean isManualControl = false;
    private boolean lastControlledManually = false;  // Track last control mode

    
    
    public OmmatophoreSubsystem(int motorPort) {
        ommatophoreMotor = new SparkMax(11, MotorType.kBrushless);
        ommatophorePID = ommatophoreMotor.getClosedLoopController();
        ommatophoreEncoder = ommatophoreMotor.getEncoder();

        // ommatophorePID.SetP(0.1);
        // ommatophorePID.setI(0.0);
        // ommatophorePID.setD(0.0);
        // ommatophorePID.setFF(0.0);

        SparkFlexConfig config = new SparkFlexConfig();

        // Set PID gains
        config.closedLoop
                .p(0.1)
                .i(0.1)
                .d(0.1)
                .outputRange(-.2, 1);
        
        ommatophoreEncoder.setPosition(0);
    }

    public void moveUp() {
        ommatophoreMotor.set(UP_SPEED);
    }

    public void moveDown() {
        ommatophoreMotor.set(DOWN_SPEED);
    }

    public void idle() {
        ommatophoreMotor.set(HOLDING_POWER);
    }

    public void stopMotor() {
        ommatophoreMotor.set(HOLDING_POWER);
    }

    public double getPosition() {
        return ommatophoreEncoder.getPosition(); // Returns encoder position
    }

    public void resetEncoder() {
        ommatophoreEncoder.setPosition(0);
    }

    public void moveUpOneStage() {
        if (currentStage < STAGES.length - 1) {
            moveToStage(currentStage + 1);
        }
    }

    public void moveDownOneStage() {
        if (currentStage > 0) {
            moveToStage(currentStage - 1);
        }
    }

    public void moveToStage(int stage) {
        if (stage >= 0 && stage < STAGES.length) {
            currentStage = stage;
            ommatophorePID.setReference(STAGES[currentStage], SparkMax.ControlType.kPosition);
            System.out.println("Moving to stage: " + currentStage + " at position " + STAGES[currentStage]);
        }
    }

    public void setManualControl(double power) {
        if (Math.abs(power) > 0.05) {  // If trigger is actively pressed
            isManualControl = true;
            ommatophoreMotor.set(power);
        } else if (isManualControl) {  // If triggers are released, stop motor
            isManualControl = false;
            ommatophoreMotor.stopMotor();  
        }

    }


    @Override
    public void periodic() {
        // double currentAmps = ommatophoreMotor.getOutputCurrent(); // Get motor current
        // SmartDashboard.putNumber("Elevator Motor Current", currentAmps); // Display on dashboard
        // System.out.println("Elevator Motor Current: " + currentAmps + " A"); // Print to console
        // You can log encoder values for debugging
        //System.out.println("Ommatophore Position: " + getPosition());
    }
}
