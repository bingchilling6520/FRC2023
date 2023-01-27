// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.SubsystemInstances.*;
import static frc.robot.Constants.ControllerMapping.*;
import static frc.robot.Constants.Sensitivity.*;
import static frc.robot.Constants.MotorCoefficients.*;
import static frc.robot.Constants.PID.*;

public class DriveJoystick extends CommandBase {
  /** Creates a new DriveJoystick. */

  // Create PID controller
  PIDController PID = new PIDController(KP, KI, KD);

  public DriveJoystick() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveBaseInstance);
    addRequirements(GyroInstance);
  }

  /** Sensitivity check */
  private double sense(double val) {
    return Math.abs(val) > JOYSTICK_SENSE ? val : 0;
  }

  /** Axis button check */
  private double boostbutton() {
    return sense(JOYSTICK0.getRawAxis(BOOST)) > 0 ? BOOST_SPEED : NORMAL_SPEED;
  }

  private boolean norotatebutton() {
    return sense(JOYSTICK0.getRawAxis(NOROTATE)) > 0;
  }

  /**
   * Simplify angle
   * If angle > 180 degree -> minus 360 degree
   * If angle < -180 degree -> plus 180 degree
  */
  private double simplifyAngle(double angle) {
    while (angle > 180) {
      angle -= 360;
    }

    while (angle < -180) {
      angle += 360;
    }

    return angle;
  }

  /** Clamp: return the max or min value if out of given range */
  private double clamp(double value, double min, double max) {
    if (value > max) {
      return max;
    }

    else if (value < min) {
      return min;
    }

    return value;
  }

  /** Rotate inplace to the target angle based on input X and Y axis values */
  private double turnToAngle(double x, double y) {
    // Calculate right joystick angle
    double angle = simplifyAngle(Math.toDegrees(Math.atan2(x, -y)));

    // Set goal for PID
    PID.setSetpoint(angle);

    // Rotate
    DriveBaseInstance.drive(0, 0, clamp(-PID.calculate(GyroInstance.getYaw()) * 0.1, -0.6, 0.6));

    return angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Normally, when initialized, set velocity to 0
    DriveBaseInstance.drive(0, 0, 0);

    // Initial PID config
    PID.setSetpoint(0); // Dummy value
    PID.enableContinuousInput(-180, 180); // Range of degree -180 -> 180
    PID.setIntegratorRange(-10, 1); // Integral range
    PID.setTolerance(KTOLERANCE, KTOLERANCEVELOCITY); // Set tolerance
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

    // Get axis values
    double leftX = sense(JOYSTICK0.getRawAxis(XAXISLEFT));
    double leftY = sense(JOYSTICK0.getRawAxis(YAXISLEFT));
    double rightX = sense(JOYSTICK0.getRawAxis(XAXISRIGHT));
    double rightY = sense(JOYSTICK0.getRawAxis(YAXISRIGHT));

    // If all axis values are 0, then this function has no use :)
    if (leftX == 0 && leftY == 0 && rightX == 0 && rightY == 0) {
      // Put SmartDashboard value
      SmartDashboard.putNumber("Left joystick angle", 999);
      SmartDashboard.putNumber("Right joystick angle", 999);

      DriveBaseInstance.drive(0, 0, 0); // Stop drivebase
      return;
    }

    // If only right joystick moves -> only rotate inplace
    if (leftX == 0 && leftY == 0 && (rightX != 0 || rightY != 0)) {
      // Turn to angle
      double angle = turnToAngle(rightX, rightY);
      
      // Put value to SmartDashboard
      SmartDashboard.putNumber("Left joystick angle", 999);
      SmartDashboard.putNumber("Right joystick angle", angle);
    }

    // If only left joystick moves -> rotate and move according to the axis value
    if ((leftX != 0 || leftY != 0) && rightX == 0 && rightY == 0) {
      // Turn to angle if no rotate button not pressed
      if (!norotatebutton()) {
        double angle = turnToAngle(leftX, leftY);

        // Put value to SmartDashboard
        SmartDashboard.putNumber("Left joystick angle", angle);
        SmartDashboard.putNumber("Right joystick angle", 999);
      }

      else {
        // Put value to SmartDashboard
        SmartDashboard.putNumber("Left joystick angle", 999);
        SmartDashboard.putNumber("Right joystick angle", 999);
      }

      // Drive according to the axis values
      SmartDashboard.putNumber("X velocity", -leftX);
      SmartDashboard.putNumber("Y velocity", -leftY);

      // Drive
      DriveBaseInstance.drive(-leftX * boostbutton(), -leftY * boostbutton(), 0);
    }

    // Reached target angle
    if (PID.atSetpoint()) {
      DriveBaseInstance.drive(0, 0, 0); // Stop drivebase
      PID.reset(); // Reset the PID controller
    }
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
