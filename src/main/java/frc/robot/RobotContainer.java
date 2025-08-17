// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveMandibleCommand;
import frc.robot.commands.apriltags.DriveToTag;
import frc.robot.commands.apriltags.PositionEstimation;
import frc.robot.commands.apriltags.TurnToTag;
import frc.robot.commands.autocommands.AutoCoral;
import frc.robot.commands.crusherclawcommands.MoveCrusherClawCommand;
import frc.robot.commands.ommatophore.MoveOmmatophoreCommand;
import frc.robot.subsystems.CrusherClawSubsystem;
import frc.robot.subsystems.MandibleSubsystem;
import frc.robot.subsystems.MaxillipedSubsystem;
import frc.robot.subsystems.OmmatophoreSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;


public class RobotContainer {

    final XboxController driverXbox = new XboxController(0);
    final GenericHID buttonBoard = new GenericHID(1);

    private final PhotonCamera camera = new PhotonCamera(Constants.AprilTags.CameraConstants.kCameraName);

    // Subsystems & Commands

    private final OmmatophoreSubsystem ommatophoreSubsystem = new OmmatophoreSubsystem(11); // CAN ID of elevator motor
    private final CrusherClawSubsystem crusherClawSubsystem = new CrusherClawSubsystem(12); // CAN ID of arm motor
    private final MandibleSubsystem mandibleSubsystem = new MandibleSubsystem(13); // CAN ID of climber motor
    private final MaxillipedSubsystem maxillipedSubsystem = new MaxillipedSubsystem(14); // CAN ID of pin motor

    private final SwerveSubsystem drivebase = new SwerveSubsystem();
    private final VisionSubsystem vision = new VisionSubsystem(camera, drivebase);

    private final PositionEstimation aprilPositionEstimation = new PositionEstimation(vision);

    private PIDController turnController = new PIDController(.1, 0, 0);
    private PIDController forwardController = new PIDController(1, 0, 1);

    private TurnToTag pointToTag = new TurnToTag(camera, drivebase, turnController, forwardController);
    private DriveToTag driveToTag = new DriveToTag(driverXbox, camera, drivebase);

    private AutoCoral autoCoral = new AutoCoral(crusherClawSubsystem, ommatophoreSubsystem);
    private SendableChooser<Command> autoChooser;
    PathPlannerAuto thisAuto = new PathPlannerAuto("Main - 2 Coral Auto");

    public RobotContainer() {

    NamedCommands.registerCommand("autoCoral", autoCoral);
        

        new EventTrigger("Score Coral").onTrue(autoCoral);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the trigger bindings
        configureBindings();
        drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(() -> driverXbox.getRightX() * 1)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(driverXbox::getRightX,
                    driverXbox::getRightY)
            .headingWhile(true);

    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

    private void configureBindings() {

        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        drivebase.setDefaultCommand(driveRobotOrientedAngularVelocity);

        new JoystickButton(driverXbox, XboxController.Button.kBack.value)              // Button to reorient the current direction as "0"
                      .onTrue((new InstantCommand(drivebase::zeroGyro)));

        Command moveCrusherClawCommand = new MoveCrusherClawCommand(crusherClawSubsystem, driverXbox);
        crusherClawSubsystem.setDefaultCommand(moveCrusherClawCommand);

        Command moveMandibleCommand = new MoveMandibleCommand(mandibleSubsystem, driverXbox);
        mandibleSubsystem.setDefaultCommand(moveMandibleCommand);

        new Trigger(() -> driverXbox.getStartButtonPressed())
        .whileTrue(new DriveToTag(driverXbox, camera, drivebase));

        // Manual elevator control (Right Trigger = Up, Left Trigger = Down)
        new Trigger(() -> driverXbox.getRightTriggerAxis() > 0.05)
                .whileTrue(new MoveOmmatophoreCommand(ommatophoreSubsystem, driverXbox));

        new Trigger(() -> driverXbox.getLeftTriggerAxis() > 0.05)
                .whileTrue(new MoveOmmatophoreCommand(ommatophoreSubsystem, driverXbox));

        // // Stage-based elevator control
        // new JoystickButton(buttonBoard, 5)
        //         .onTrue(new MoveOmmatophoreStageCommand(ommatophoreSubsystem, 0)); // Bottom stage

        // new JoystickButton(buttonBoard, 3)
        //         .onTrue(new MoveOmmatophoreStageCommand(ommatophoreSubsystem, 1)); // Middle stage

        // new JoystickButton(buttonBoard, 4)
        //         .onTrue(new MoveOmmatophoreStageCommand(ommatophoreSubsystem, 2)); // Top stage

        // Maxilliped Control
        // new JoystickButton(driverXbox, XboxController.Button.kStart.value)
        //         .onTrue(new MaxillipedCommand(maxillipedSubsystem));

        // Camera Control
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
