// MOVES THE CRUSHER CLAW (THE GRABBER)

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CrusherClawSubsystem;
import edu.wpi.first.wpilibj.XboxController;

public class MoveCrusherClawCommand extends Command {
    private final CrusherClawSubsystem crusherClawSubsystem;
    private final XboxController controller;

    public MoveCrusherClawCommand(CrusherClawSubsystem crusherClawSubsystem, XboxController driverXbox) {
        this.crusherClawSubsystem = crusherClawSubsystem;
        this.controller = driverXbox;
        addRequirements(crusherClawSubsystem);
    }

    @Override
    public void execute() {
        if (controller.getRawButton(XboxController.Button.kRightBumper.value)) {
            crusherClawSubsystem.moveUp();  // Right bumper moves up
        } else if (controller.getRawButton(XboxController.Button.kLeftBumper.value)) {
            crusherClawSubsystem.moveDown(); // Left bumper moves down
        } else {
            crusherClawSubsystem.stop();
        }
    
   
   
    
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
        crusherClawSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted
    }
}