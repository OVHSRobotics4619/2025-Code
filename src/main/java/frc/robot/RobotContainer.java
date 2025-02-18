// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveCrusherClawCommand;
import frc.robot.commands.MoveMandibleCommand;
import frc.robot.commands.ommatophore.MoveOmmatophoreCommand;
import frc.robot.commands.ommatophore.MoveOmmatophoreStageCommand;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.subsystems.CrusherClawSubsystem;
import frc.robot.subsystems.MandibleSubsystem;
import frc.robot.subsystems.OmmatophoreSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  
  final XboxController driverXbox = new XboxController(0);
  final GenericHID buttonBoard = new GenericHID(1);
  
  // The robot's subsystems and commands are defined here...
  private final OmmatophoreSubsystem ommatophoreSubsystem = new OmmatophoreSubsystem(11); // CAN ID of elevator motor
  private final CrusherClawSubsystem crusherClawSubsystem = new CrusherClawSubsystem(12); // CAN ID of elevator motor
  private final MandibleSubsystem mandibleSubsystem = new MandibleSubsystem(13); // CAN ID of climber motor
  

  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/neo"));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();   
  }
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.4)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary predicate, or via the
   * named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
   * Flight joysticks}.
   */
  private void configureBindings() {

    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

    Command moveCrusherClawCommand = new MoveCrusherClawCommand(crusherClawSubsystem, driverXbox);

    crusherClawSubsystem.setDefaultCommand(moveCrusherClawCommand);

    
    Command moveMandibleCommand = new MoveMandibleCommand(mandibleSubsystem, driverXbox);

    mandibleSubsystem.setDefaultCommand(moveMandibleCommand);

    // Manual elevator control (Right Trigger = Up, Left Trigger = Down)
        new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.05)
        .whileTrue(new MoveOmmatophoreCommand(ommatophoreSubsystem,driverXbox));

        new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.05)
        .whileTrue(new MoveOmmatophoreCommand(ommatophoreSubsystem, driverXbox));

    // // Stage-based elevator control (B = Up one stage, X = Down one stage)
    //     new JoystickButton(driverXbox, XboxController.Button.kB.value)
    //         .onTrue(new MoveOmmatophoreStageCommand(ommatophoreSubsystem, driverXbox));

    //     new JoystickButton(driverXbox, XboxController.Button.kX.value)
    //         .onTrue(new MoveOmmatophoreStageCommand(ommatophoreSubsystem, driverXbox));

    

    

    // driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    // driverXbox.b().whileTrue(
    //     drivebase.driveToPose(
    //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
    // driverXbox.start().whileTrue(Commands.none());
    // driverXbox.back().whileTrue(Commands.none());
    // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    // driverXbox.rightBumper().onTrue(Commands.none());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
    // // An example command will be run in autonomous
    // return drivebase.getAutonomousCommand("New Auto");
  // }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
