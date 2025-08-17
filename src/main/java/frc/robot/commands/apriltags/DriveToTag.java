package frc.robot.commands.apriltags;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class DriveToTag extends Command {

    private final PhotonCamera camera;
    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;

    private boolean ended = false;

    private final double VISION_TURN_kP = 0.01;
    private final double VISION_DES_ANGLE_deg = 0.0;
    private final double VISION_STRAFE_kP = 0.5;
    private final double VISION_DES_RANGE_m = 1.25;

    public DriveToTag(XboxController controller, PhotonCamera camera, SwerveSubsystem swerveSubsystem) {
        this.camera = camera;
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        ended = false;
    }

    @Override
    public void execute() {

        // Calculate drivetrain commands from Joystick values
        double forward = -controller.getLeftY() * Constants.maxSpeed;
        double strafe = -controller.getLeftX() * Constants.maxSpeed;
        double turn = controller.getRightX() * Constants.maxRotationSpeed;

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        double targetRange = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 18) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                            Units.inchesToMeters(15.25), // Measured with a tape measure, or in CAD.
                            Units.inchesToMeters(7), // From 2025 measurement
                                Units.degreesToRadians(0.0), // Measured with a protractor, or in CAD.
                                Units.degreesToRadians(target.getPitch()));


                        System.out.printf("Target Yaw %.2f; Range %.2f\n", targetYaw, targetRange);

                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (controller.getStartButtonPressed() && targetVisible) {
            System.out.println("Target is visible!");
            // Driver wants auto-alignment to tag 18
            // And, tag 18 is in sight, so we can turn toward it.
            // Override the driver's turn and fwd/rev command with an automatic one
            // That turns toward the tag, and gets the range right.
            turn = (VISION_DES_ANGLE_deg - targetYaw) * VISION_TURN_kP * Constants.maxRotationSpeed;
            forward = (VISION_DES_RANGE_m - targetRange) * VISION_STRAFE_kP * Constants.maxSpeed;
            System.out.printf("Driving forward %.2f; turning %.2f\n", forward, turn);

            // Command drivetrain motors based on target speeds
            swerveSubsystem.drive(forward, strafe, turn);
        }

        
    }

    @Override
    public void end(boolean interrupted) {
        ended = true;
    }

    @Override
    public boolean isFinished() {
        System.out.printf("isFinished: %b", ended);
        return ended;
    }
}