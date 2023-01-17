// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;

import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.Gyro;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /** Controller ensitivity */
  public static class Sensitivity {
    /** Joystick sensitivity */
    public static final double JOYSTICK_SENSE = 0.15;
  }

  /** Motor coefficients */
  public static class MotorCoefficients {
    /** Normal speed coefficient */
    public static double NORMAL_SPEED = 0.4;
    /** Boosted speed coefficient */
    public static double BOOST_SPEED = 0.8;
  }

  /** CAN devices ID */
  public static class CAN_ID {
    /** Left Front Motor */
    public static final int LEFT_FRONT = 1;
    /** Left Back Motor */
    public static final int LEFT_BACK = 2;
    /** Right Front Motor */
    public static final int RIGHT_FRONT = 3;
    /** Right Back Motor */
    public static final int RIGHT_BACK = 4;
  }

  /** Kinematics Meters */
  public static class KinematicsMeters {
    /** Left Front Wheel */
    public static Translation2d LEFT_FRONT_CENTER = new Translation2d();
    /** Left Back Wheel */
    public static Translation2d LEFT_BACK_CENTER = new Translation2d();
    /** Right Front Wheel */
    public static Translation2d RIGHT_FRONT_CENTER = new Translation2d();
    /** Right Back Wheel */
    public static Translation2d RIGHT_BACK_CENTER = new Translation2d();
  }

  /** Controller button mapping */
  public static class ControllerMapping {
    // Joysticks
    
    /** First Joystick */
    public static Joystick JOYSTICK0 = new Joystick(0);
    /** Second Joystick */
    public static Joystick JOYSTICK1 = new Joystick(1);

    // Axises

    /** X axis of the left joystick */
    public static int XAXISLEFT = 0;
    /** Y axis of the left joystick */
    public static int YAXISLEFT = 1;
    /** X axis of the right joystick */
    public static int XAXISRIGHT = 4;
    /** Y axis of the right joystick */
    public static int YAXISRIGHT = 5;

    // Buttons
    /** Boost speed button */
    public static int BOOST = 4;
  }

  /**
   * Subsystem instances
   * It's surely stupid to do this, but Roborio doesn't have that much RAM, so we need to
   * reduce the memory usage by creating only one instance of a subsystem. Also we want
   * unification of subsystem instances :)  
   * 
   * Idea credit: KhiemGOM :)
  */
  public static class SubsystemInstances {
    /** Drivebase subsystem instance */
    public static Drivebase DriveBaseInstance = new Drivebase();
    /** Gyro subsystem instance */
    public static Gyro GyroInstance = new Gyro();
  }
}
