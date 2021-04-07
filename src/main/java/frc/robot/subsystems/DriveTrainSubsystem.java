// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

public class DriveTrainSubsystem extends SubsystemBase {

  public WPI_TalonFX rightMain;
  public WPI_TalonFX rightFollow;
  public WPI_TalonFX leftMain;
  public WPI_TalonFX leftFollow;

  public PigeonIMU pigeon;

  public DifferentialDrive twoMotorDrive;

  private final DifferentialDriveOdometry odometry;


  public boolean isReversed = false;


public DriveTrainSubsystem() {
  leftMain = new WPI_TalonFX((11));
  leftFollow = new WPI_TalonFX((12));
  rightMain = new WPI_TalonFX((21));
  rightFollow = new WPI_TalonFX((22));

  pigeon = new PigeonIMU(0);

  leftMain.setInverted(true);
  leftFollow.setInverted(true);
  rightMain.setInverted(true);
  rightFollow.setInverted(true);

  leftFollow.follow(leftMain);
  rightFollow.follow(rightMain);

  leftMain.setNeutralMode(NeutralMode.Coast);
  leftFollow.setNeutralMode(NeutralMode.Coast);
  rightMain.setNeutralMode(NeutralMode.Coast);
  rightFollow.setNeutralMode(NeutralMode.Coast);

  twoMotorDrive = new DifferentialDrive(leftMain, rightMain);

  resetEncoders();
    odometry = new DifferentialDriveOdometry(getHeading());
}



  @Override
  public void periodic() {

    SmartDashboard.putNumber("Left Position", leftMain.getSelectedSensorPosition());
    SmartDashboard.putNumber("Right Position", rightMain.getSelectedSensorPosition());
    SmartDashboard.putNumber("PosX", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("PosY", odometry.getPoseMeters().getTranslation().getY());


    SmartDashboard.putNumber("Distance (meters)", (leftMain.getSelectedSensorPosition()*(ENCODER_DISTANCE_METERS_PER_PULSE)));
    SmartDashboard.putNumber("Left Distance (meters)", -1*getMotorPositionsInMeters(leftMain));
    SmartDashboard.putNumber("Right Distance (meters)", getMotorPositionsInMeters(rightMain));
 
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("Turn Rate", getTurnRate());
    SmartDashboard.putNumber("Left Velocity (m/s)", -1*leftMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE*10);
    SmartDashboard.putNumber("Right Velocity (m/s)", rightMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE*10);

    SmartDashboard.putNumber("Pigeon Yaw Pitch Roll", getPigeonYawPitchRoll()[0]);
   

    //odometry.update(getHeading(), -1*getMotorPositionsInMeters(leftMain), getMotorPositionsInMeters(rightMain));
  }

  public void driveWithJoystick(double left, double right) {

    // if not reversed, normal tank drive, else flip left and right and negate the values
    if (isReversed == false) {
      twoMotorDrive.tankDrive(left, right);
    } else {
      twoMotorDrive.tankDrive(-right, -left);
    }

  }

  public void driveWithArcade(double speed, double rotation) {
    twoMotorDrive.arcadeDrive(speed, rotation*-1);
  }

  public void driveForward(double left, double right) {
    twoMotorDrive.tankDrive(left, right);
  }

  public void resetEncoders() {

    rightMain.setSelectedSensorPosition(0);
    leftMain.setSelectedSensorPosition(0);

  }

  public double getMotorPositionsInMeters(WPI_TalonFX motor) {
    return motor.getSelectedSensorPosition() * ENCODER_DISTANCE_METERS_PER_PULSE;
  }

  public Rotation2d getHeading() {

    double[] ypr = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    //TODO:                                                       V Might be multiplied by negative 1
    return Rotation2d.fromDegrees(Math.IEEEremainder(ypr[0], 360.0));


  }
  
  public void resetHeading() {
    pigeon.setYaw(0.0);
  }

  // public DifferentialDriveKinematics getKinematics() {
  //   return kinematics;
  // }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-1*leftMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE * 10, rightMain.getSelectedSensorVelocity()*ENCODER_DISTANCE_METERS_PER_PULSE * 10);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMain.setVoltage(-leftVolts);
    rightMain.setVoltage(rightVolts);
    twoMotorDrive.feed();
  }

  public double getAverageEncoderDistance() {
    return (getMotorPositionsInMeters(leftMain) + (-1 * getMotorPositionsInMeters(rightMain))) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    twoMotorDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    pigeon.setYaw(0);
  }

  public double getTurnRate() {
    double[] xyz_dps = {0,0,0};
    pigeon.getRawGyro(xyz_dps);
    return xyz_dps[0];
  }

  public double[] getPigeonYawPitchRoll() { 
    double[] ypr = {0,0,0};
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

}
