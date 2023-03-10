// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */

  private AHRS ahrs = new AHRS();

  public Gyro() {}

  /** Reset Gyro */
  public void reset() {
    ahrs.reset();
  }

  /** getYaw */
  public double getYaw() {
    return ahrs.getYaw();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    float vx = ahrs.getVelocityX();
    float vy = ahrs.getVelocityY();
    float vz = ahrs.getVelocityZ();

    SmartDashboard.putNumber("X velocity", vx);
    SmartDashboard.putNumber("Y velocity", vy);
    SmartDashboard.putNumber("Z velocity", vz);
    SmartDashboard.putNumber("Angle", ahrs.getYaw());
    SmartDashboard.putNumber("NavX temperature", ahrs.getTempC());
  }
}
