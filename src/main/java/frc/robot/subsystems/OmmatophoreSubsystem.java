package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class OmmatophoreSubsystem extends SubsystemBase {
    private final SparkMax ommatophoreMotor;
    private final RelativeEncoder ommatophoreEncoder;
    private final SparkClosedLoopController ommatophorePID;

    private static final double HOLDING_POWER = 0.01; // Power needed to prevent elevator from falling naturally, but not enough to move
    private static final double STOP_SPEED = 0.0; // Stop
    private static final double POSITION_TOLERANCE = 2.0; // Allowable error in encoder units
    private static final double MAX_MANUAL_SPEED = 0.2; // Up and down speed

    public static final double[] STAGES = {55.0, 106.0, 215.0}; // Encoder positions for each stage
    private int currentStage = 0; // Track the current stage
    private boolean isManualControl = false; // Flag for manual control

    public OmmatophoreSubsystem(int motorPort) {
        ommatophoreMotor = new SparkMax(motorPort, MotorType.kBrushless);
        ommatophorePID = ommatophoreMotor.getClosedLoopController();
        ommatophoreEncoder = ommatophoreMotor.getEncoder();

        SparkFlexConfig config = new SparkFlexConfig();
        config.closedLoop
                .p(0.1)
                .i(0.0)
                .d(0.0)
                .outputRange(-0.3, 0.3);

        ommatophoreEncoder.setPosition(0);
    }

    public void moveToStage(int stageIndex) {
        if (stageIndex >= 0 && stageIndex < STAGES.length) {
            double targetPosition = STAGES[stageIndex];
            ommatophorePID.setReference(targetPosition, SparkMax.ControlType.kPosition);
            currentStage = stageIndex;
            System.out.println("Moving to stage: " + stageIndex + " at position " + targetPosition);
        }
    }

    public void setManualControl(double power) {
        double currentPosition = ommatophoreEncoder.getPosition();

        // Enforce soft limits
        if ((power > 0 && currentPosition >= STAGES[2]) || // Prevent moving above stage 2
            (power < 0 && currentPosition <= STAGES[0])) { // Prevent moving below stage 0
            power = 0;
        }

        if (Math.abs(power) > 0.05) { // Deadband threshold
            isManualControl = true;
            ommatophoreMotor.set(Math.copySign(Math.min(Math.abs(power), MAX_MANUAL_SPEED), power));
        } else if (isManualControl) { // Stop motor when triggers are released
            isManualControl = false;
            stopMotor();
        }
    }

    public void stopMotor() {
        ommatophoreMotor.set(HOLDING_POWER);
    }

    public double getPosition() {
        return ommatophoreEncoder.getPosition();
    }

    public double[] getStagePositions() {
        return STAGES;
    }

    @Override
    public void periodic() {
        // System.out.println("Ommatophore Position: " + getPosition());
        // SmartDashboard.putNumber("Ommatophore Position", getPosition());
    }

    public void moveUp() {
        ommatophoreMotor.set(MAX_MANUAL_SPEED);
    }

    public void moveDown() {
        ommatophoreMotor.set(-MAX_MANUAL_SPEED);
    }

    public void idle() {
        ommatophoreMotor.set(HOLDING_POWER);
    }
}