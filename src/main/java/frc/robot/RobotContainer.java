// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static frc.robot.Constants.ControllerMapping.*;
import frc.robot.commands.GotoAprilTag;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Buttons
  private final JoystickButton driveToScoreButton = new JoystickButton(JOYSTICK0, DRIVE_TO_SCORE);
  private final JoystickButton driveToSubstationButton = new JoystickButton(JOYSTICK0, DRIVE_TO_SUBSTATION);

  // Commands
  private final GotoAprilTag driveToScore = new GotoAprilTag(-1); // Choose best apriltag

  /*
   * The AprilTag ID in the Blue's substation is 4, while the AprilTag ID in the Red's is 5
   * Get the current team color (set in DriverStation), then decide the correct AprilTag ID to aim 
   */
  private final GotoAprilTag driveToSubstation
    = new GotoAprilTag(DriverStation.getAlliance() == Alliance.Blue ? 4 : 5);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    driveToScoreButton.whileTrue(driveToScore);
    driveToSubstationButton.whileTrue(driveToSubstation);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
