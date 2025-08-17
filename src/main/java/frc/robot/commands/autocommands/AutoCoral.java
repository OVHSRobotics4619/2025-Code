package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.crusherclawcommands.LowerCrusherClawCommand;
import frc.robot.commands.crusherclawcommands.RaiseCrusherClawCommand;
import frc.robot.commands.ommatophore.MoveOmmatophoreStageCommand;
import frc.robot.subsystems.CrusherClawSubsystem;
import frc.robot.subsystems.OmmatophoreSubsystem;

public class AutoCoral extends SequentialCommandGroup {
    public AutoCoral(CrusherClawSubsystem crusherClaw, OmmatophoreSubsystem ommatophore) {
        addCommands(
            // Step 1: Raise the Crusher Claw
            new RaiseCrusherClawCommand(crusherClaw),

            // Step 2: Move Ommatophore to top stage
            new MoveOmmatophoreStageCommand(ommatophore, 2), // Assuming stage 2 is the top stage

            // Step 3: Swing the Crusher Claw down
            new LowerCrusherClawCommand(crusherClaw)
        );
    }
}