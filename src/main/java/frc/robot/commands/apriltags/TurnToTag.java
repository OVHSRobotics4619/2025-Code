package frc.robot.commands.apriltags;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class TurnToTag extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerveSubsystem;
    private final PIDController angleCalculator;
    private final PIDController distanceCalculator;

    private boolean ended = false;

    public TurnToTag(PhotonCamera camera, SwerveSubsystem swerveSubsystem, PIDController angle, PIDController forward) {
        this.camera = camera;
        this.angleCalculator = angle;
        this.distanceCalculator = forward;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        PhotonPipelineResult result = camera.getLatestResult();
        double angle = -1;
        double distance = -1;

        System.out.println("Looking for april tags");

        if (result.hasTargets()) {

            List<PhotonTrackedTarget>targets = result.getTargets();
            for (PhotonTrackedTarget target : targets) 
            {
                int targetId = target.getFiducialId();
                if (targetId == 18)
                {
                    System.out.println("Found april tag 18");

                    double yaw = target.getYaw();

                    angle = angleCalculator.calculate(yaw, 0);

                    Transform3d cameraTarget = target.getBestCameraToTarget();

                    PhotonUtils.estimateCameraToTarget(null, null, null);

                    double range = cameraTarget.getX();
                    double yValue = cameraTarget.getY();
                    distance = distanceCalculator.calculate(range, Constants.GOAL_RANGE_METERS);
                    System.out.println("Yaw: " + yaw);
                    System.out.println("Angle: " + angle);
                    System.out.println("Range: " + range);
                    System.out.println("Y Value: " + yValue);
                    System.out.println("Distance: " + distance);

                    swerveSubsystem.driveWithVision(angle, distance);
                    break;
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // ended = true;
    }

    @Override
    public boolean isFinished() {
        return ended;
    }
}