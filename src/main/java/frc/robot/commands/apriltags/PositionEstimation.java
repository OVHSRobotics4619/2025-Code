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

    }

    @Override
    public void execute() {
        vision.updateGlobalPosition();
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
   
        return false;
    }
}