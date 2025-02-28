package frc.robot.commands.ommatophore;

import frc.robot.subsystems.OmmatophoreSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveOmmatophoreStageCommand extends Command {
    private final OmmatophoreSubsystem ommatophore;
    private final int targetStage;

    public MoveOmmatophoreStageCommand(OmmatophoreSubsystem ommatophore, int targetStage) {
        this.ommatophore = ommatophore;
        this.targetStage = targetStage;
        addRequirements(ommatophore);
    }

    @Override
    public void initialize() {
        System.out.println("Moving Ommatophore to Stage: " + targetStage);
        ommatophore.moveToStage(targetStage);
    }

    @Override
    public boolean isFinished() {
        // Check if the elevator is close enough to the target position
        double currentPos = ommatophore.getPosition();
        double targetPos = OmmatophoreSubsystem.STAGES[targetStage];
        double tolerance = 2.0; // Allowable error range

        return Math.abs(currentPos - targetPos) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Ommatophore Stage Movement Interrupted!");
        } else {
            System.out.println("Ommatophore Reached Stage: " + targetStage);
        }
        ommatophore.stopMotor();
    }
}