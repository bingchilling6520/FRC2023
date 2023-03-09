// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.SubsystemInstance.*;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  public AutoBalance() {
    addRequirements(m_Drivebase);
    addRequirements(m_Gyro);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_Gyro.getPitch() * 0.05;
    m_Drivebase.driveWithField(speed, 0, 0, m_Gyro.getRotation2d());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Drivebase.driveWithField(0, 0, 0, m_Gyro.getRotation2d());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
