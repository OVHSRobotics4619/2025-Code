package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.MaxillipedSubsystem;

public class MaxillipedCommand extends InstantCommand {
    private final MaxillipedSubsystem maxillipedSubsystem;

    public MaxillipedCommand(MaxillipedSubsystem subsystem) {
        super(subsystem::toggleMaxilliped);
        this.maxillipedSubsystem = subsystem;
    }
}