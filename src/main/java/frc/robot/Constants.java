// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ENCODER_DISTANCE_METERS_PER_PULSE = (.1524 * 3.14159265)/29020.16;

    public static final double OUTER_INTAKE_MOTOR_SPEED = .65;
	public static final double INNER_INTAKE_MOTOR_SPEED = 1;
	public static final double CONVEYOR_MOTOR_SPEED = .6;
	public static final double STOP_MOTOR = 0;

	public static final double TOF_SAMPLE_TIME = 25;

	public static final int kTimeoutMs = 30;
	public static final int kPIDLoopIdx = 0;

	//public static final Gains kGains_Velocit = new Gains(0.049, 0.1, 0.0, 0.0);
	public static final double DRIVE_FORWARD_TIME = 3;
	public static final double AUTONOMOUS_SPEED = .5;

	public static final double STEER_K = 0.03;
	public static final double DRIVE_K = 0.26;
	public static final double DESIRED_TARGET_AREA = 13.0;
	public static final double MAX_DRIVE = 0.7;

	public static final double LIMELIGHT_DEADBAND = 1;
    public static final double MIN_STEER_K = .4;
    

    // Constants for the odometry stuff
    public static final double kS = 0.4;
    public static final double kV = 4.09;
    public static final double kA = 0.202;
    public static final double kP = 0.0000156;
    public static final double TRACK_WIDTH = .687922496;


    //Constants for PathWeaver
    public static final double DRIVE_TRAIN_WIDTH = 0.678;
	public static final double GEAR_RATIO = 10.86;
	public static final double WHEEL_RADIUS = 0.1016;

	// TODO: this might be in feet, could have to put in meters
	public static final double KS = 0.671;
	public static final double KV = 2.53;
	public static final double KA = 0.136;
	// r^2 .999

	public static final double KAVoltSecondsSquaredPerMeter = 0.0;
  
    public static final DifferentialDriveKinematics K_DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

	public static final double kMaxSpeedMetersPerSecond = 3.1;
	public static final double kMaxAccelerationMetersPerSecondSquared = 3.1;

	public static final double kRamseteB = 2.0;
	public static final double kRamseteZeta = 0.7;
	public static final double KP = 0.000103;
}
