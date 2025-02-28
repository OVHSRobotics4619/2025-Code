package frc.robot.commands.crusherclawcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CrusherClawSubsystem;

public class LowerCrusherClawCommand extends Command {
    private final CrusherClawSubsystem crusherClaw;

    public LowerCrusherClawCommand(CrusherClawSubsystem crusherClaw) {
        this.crusherClaw = crusherClaw;
        addRequirements(crusherClaw);
    }

    @Override
    public void initialize() {
        crusherClaw.moveToPosition(CrusherClawSubsystem.ARM_DOWN_POSITION);
    }

    @Override
    public boolean isFinished() {
        return crusherClaw.isAtPosition(CrusherClawSubsystem.ARM_DOWN_POSITION);
    }
}