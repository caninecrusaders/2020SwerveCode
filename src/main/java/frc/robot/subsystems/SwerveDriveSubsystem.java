/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSubsystem extends SubsystemBase {
  public static final double WHEELBASE = 18; 
  public static final double TRACKWIDTH = 18; 

  public static final double WIDTH = 25;
  public static final double LENGTH = 25; 
  public static boolean enableDrive = true;
//holonomicdrivetrain
  private double mAdjustmentAngle = 0;
  private boolean mFieldOriented = true;
//drivetrain

  private double speedMultiplier = 1;
//swerve module
  private boolean driveInverted = false;

  private SwerveModule[] mSwerveModules;

  private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);

  public SwerveDriveSubsystem() {
    zeroGyro();

    mSwerveModules = new SwerveModule[] {
        new SwerveModule(1, Constants.angleMotorFR, Constants.driveMotorFR, Constants.encoderFR),
        new SwerveModule(2, Constants.angleMotorFL, Constants.driveMotorFL, Constants.encoderFL),
        new SwerveModule(3, Constants.angleMotorBL, Constants.driveMotorBL, Constants.encoderBL),
        new SwerveModule(4, Constants.angleMotorBR, Constants.driveMotorBR, Constants.encoderBR)

    };
    mSwerveModules[0].setDriveInverted(true);
    mSwerveModules[3].setDriveInverted(true);

    for (SwerveModule module : mSwerveModules) {
      module.setTargetAngle(0);
      module.setDriveGearRatio(5.7777);
      module.setDriveWheelRadius(module.getDriveWheelRadius() * 1.05);
      module.setMotionConstraints(getMaxAcceleration(), getMaxVelocity());
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDriveInverted(boolean inverted) {
    driveInverted = inverted;
  }  

  public void setAdjustmentAngle(double adjustmentAngle) {
    System.out.printf("New adjustment Angle: % .3f\n", adjustmentAngle);
    mAdjustmentAngle = adjustmentAngle;
  }

  public double getRawGyroAngle() {
    double angle = mNavX.getAngle();
    angle %= 360;
    if (angle < 0)
      angle += 360;

    return angle;
  }

  public void zeroGyro() {
    setAdjustmentAngle(getRawGyroAngle());
  }
}
