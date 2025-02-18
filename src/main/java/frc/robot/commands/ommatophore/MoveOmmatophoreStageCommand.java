package frc.robot.commands.ommatophore;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OmmatophoreSubsystem;

public class MoveOmmatophoreStageCommand extends Command {
    private final OmmatophoreSubsystem ommatophoreSubsystem;
    private final XboxController controller;

    public MoveOmmatophoreStageCommand(OmmatophoreSubsystem ommatophoreSubsystem, XboxController driverXbox) {
        this.ommatophoreSubsystem = ommatophoreSubsystem;
        this.controller = driverXbox;
        addRequirements(ommatophoreSubsystem);
    }

    @Override
    public void execute() {
        if (controller.getBButton()) {  // Move elevator up one stage
            ommatophoreSubsystem.moveUpOneStage();
        } else if (controller.getXButton()) {  // Move elevator down one stage
            ommatophoreSubsystem.moveDownOneStage();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // No need to stop motor here, since manual control is handled separately
    }

    @Override
    public boolean isFinished() {
        return false;  // This command runs indefinitely while the buttons are being pressed
    }
}