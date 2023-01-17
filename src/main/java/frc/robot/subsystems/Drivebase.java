// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CAN_ID.*;
import static frc.robot.Constants.KinematicsMeters.*;

import java.util.ArrayList;
import java.util.List;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivebase. */

  // Talon
  private WPI_TalonSRX leftFront = new WPI_TalonSRX(LEFT_FRONT);
  private WPI_TalonSRX leftBack = new WPI_TalonSRX(LEFT_BACK);
  private WPI_TalonSRX rightFront = new WPI_TalonSRX(RIGHT_FRONT);
  private WPI_TalonSRX rightBack = new WPI_TalonSRX(RIGHT_BACK);

  // Mecanum controller
  private MecanumDrive mecanum = new MecanumDrive(leftFront, leftBack, rightFront, rightBack);
  private MecanumDriveKinematics kinematics = new MecanumDriveKinematics(LEFT_FRONT_CENTER, RIGHT_FRONT_CENTER, LEFT_BACK_CENTER, RIGHT_BACK_CENTER);

  // Encoder values list
  private List<Double> encoder = new ArrayList<>();

  public Drivebase() {
    // Disable inverted on left motors
    leftFront.setInverted(false);
    leftBack.setInverted(false);

    // Set encoder type
    leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    leftBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightBack.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Set brake mode
    leftFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);
  }

  /** Drive in the Cartesian style */
  public void drive(double x, double y, double zR) {
    mecanum.driveCartesian(x, y, zR);
  }

  /** Return encoder values list */
  public List<Double> getEncoder() {
    return encoder;
  }

  /** Return kinematics value list */
  public MecanumDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Constantly update the encoder value to the list
    encoder.add(0, leftFront.getSelectedSensorPosition());
    encoder.add(1, rightFront.getSelectedSensorPosition());
    encoder.add(2, leftBack.getSelectedSensorPosition());
    encoder.add(3, rightBack.getSelectedSensorPosition());
  }
}
