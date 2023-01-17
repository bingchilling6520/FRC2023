// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.Constants.SubsystemInstances.*;
import static frc.robot.Constants.ControllerMapping.*;
import static frc.robot.Constants.Sensitivity.*;
import static frc.robot.Constants.MotorCoefficients.*;

public class DriveJoystick extends CommandBase {
  /** Creates a new DriveJoystick. */

  public DriveJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBaseInstance);
  }

  /** Sensitivity check */
  private double sense(double val) {
    return Math.abs(val) > JOYSTICK_SENSE ? val : 0;
  }

  /** Axis button check */
  private double boostbutton() {
    return sense(JOYSTICK0.getRawAxis(BOOST)) > 0 ? BOOST_SPEED : NORMAL_SPEED;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Normally, when initialized, set velocity to 0
    DriveBaseInstance.drive(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * Something to note:
     * Playstation controllers have Y axis negative when moving joystick up
     * and X axis negative when moving joystick to the left size
     * 
     * driveCartesian() have driving up speed set to positive, driving left
     * speed set to positive and zR (rotation) counter clockwise set to
     * positive
     * 
     * So, we inverted 3 values before passing to drive()
     */

    DriveBaseInstance.drive(
      sense(-JOYSTICK0.getRawAxis(YAXISLEFT))  * boostbutton(),
      sense(-JOYSTICK0.getRawAxis(XAXISLEFT))  * boostbutton(),
      sense(-JOYSTICK0.getRawAxis(XAXISRIGHT)) * boostbutton()
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // When finished, set velocity to 0
    DriveBaseInstance.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
