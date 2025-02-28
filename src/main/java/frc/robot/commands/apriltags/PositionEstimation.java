package frc.robot.commands.apriltags;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;

public class PositionEstimation extends Command {

    private final VisionSubsystem vision;

    public PositionEstimation(VisionSubsystem vision) {
        this.vision = vision;
    }

    @Override
    public void initialize() {
        // Initialization logic (if needed)
    }

    @Override
    public void execute() {
        vision.updateGlobalPosition();
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup logic (if needed)
    }

    @Override
    public boolean isFinished() {
        // Determine when the command should end (e.g., after a certain duration)
        return false;
    }
}