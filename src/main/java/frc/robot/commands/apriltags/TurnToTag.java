package frc.robot.commands.apriltags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class TurnToTag extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController angleCalculator;
    private final PIDController distanceCalculator;

    public TurnToTag(PhotonCamera camera, SwerveSubsystem swerveSubsystem, PIDController angle, PIDController forward) {
        this.camera = camera;
        this.angleCalculator = angle;
        this.distanceCalculator = forward;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Initialization logic (if needed)
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        double angle = 0.0;
        double distance = 0.0;
        if (result.hasTargets()) {

            List<PhotonTrackedTarget>targets = result.getTargets();
            for (PhotonTrackedTarget target : targets) 
            {
                int targetId = target.getFiducialId();
                if (targetId == 18)
                {
                    angle = angleCalculator.calculate(target.getYaw(), 0);
                    double range = target.getBestCameraToTarget().getX();
                    distance = distanceCalculator.calculate(range, Constants.GOAL_RANGE_METERS);
                    // System.out.println("Forward speed: " + forwardSpeed);
                    // System.out.println("Forward distance: " + range);
                    swerveSubsystem.driveWithVision(angle, distance);
                    break;
                }
            }
        }
        
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