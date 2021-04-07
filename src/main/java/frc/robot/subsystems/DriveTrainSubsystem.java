// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveTrainSubsystem extends SubsystemBase {

  public WPI_TalonFX rightMain;
  public WPI_TalonFX rightFollow;
  public WPI_TalonFX leftMain;
  public WPI_TalonFX leftFollow;

  public AHRS navx;

  public DifferentialDrive twoMotorDrive;


  public boolean isReversed = false;


public DriveTrainSubsystem() {
  leftMain = new WPI_TalonFX((11));
  leftFollow = new WPI_TalonFX((12));
  rightMain = new WPI_TalonFX((21));
  rightFollow = new WPI_TalonFX((22));

  navx = new AHRS(SPI.Port.kMXP);

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
}



  @Override
  public void periodic() {

    SmartDashboard.putNumber("Yaw", navx.getYaw());
   
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

  public double getNavXYaw() {
    return navx.getYaw();
  }

}
