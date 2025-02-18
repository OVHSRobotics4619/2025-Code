// MOVES THE MANDIBLE (THE CLIMBER)

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MandibleSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class MoveMandibleCommand extends Command {
    private final MandibleSubsystem mandibleSubsystem;
    private final XboxController controller;

    public MoveMandibleCommand(MandibleSubsystem mandibleSubsystem, XboxController driverXbox) {
        this.mandibleSubsystem = mandibleSubsystem;
        this.controller = driverXbox;
        addRequirements(mandibleSubsystem);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(XboxController.Button.kY.value)) {
            mandibleSubsystem.moveClockwise();  // Y button moves clockwise
        } else if (controller.getRawButton(XboxController.Button.kA.value)) {
            mandibleSubsystem.moveCounterClockwise(); // A button moves counter-clockwise
        } else {
            mandibleSubsystem.stop();
        }
    }
    

    @Override
    public void end(boolean interrupted) {
        mandibleSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}