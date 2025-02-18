// MOVES THE OMMATOPHORE (THE ELEVATOR)

package frc.robot.commands.ommatophore;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.OmmatophoreSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class MoveOmmatophoreCommand extends Command {
    private final OmmatophoreSubsystem ommatophoreSubsystem;
    private final XboxController controller;

    public MoveOmmatophoreCommand(OmmatophoreSubsystem ommatophoreSubsystem, XboxController driverXbox) {
        this.ommatophoreSubsystem = ommatophoreSubsystem;
        this.controller = driverXbox;
        addRequirements(ommatophoreSubsystem);
    }

    @Override
    public void execute() {
        if (controller.getRawAxis(3)>0) {
            ommatophoreSubsystem.moveUp();  // Right trigger moves up
        } else if (controller.getRawAxis(2)>0) {
            ommatophoreSubsystem.moveDown(); // Left trigger moves down
        }
          else 
          ommatophoreSubsystem.idle();
    
   
   
    
    //if (controller.getHID.getRawButton(XboxController.Button.kX.value) != null) {
    //     System.out.println("Moving Up");
    //     elevatorSubsystem.moveUp();
    // } else if (controller.getRawButton(XboxController.Button.kY.value) != null) {
    //     System.out.println("Moving Down");
    //     elevatorSubsystem.moveDown();
    // } else {
    //     System.out.println("Stopping");
    //     elevatorSubsystem.stop();
    }
    

    @Override
    public void end(boolean interrupted) {
        ommatophoreSubsystem.idle();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}