/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public double getSpeedMultiplier() {
    return speedMultiplier;
  }

  public double getAdjustmentAngle() {
    return mAdjustmentAngle;
  }

  public double getGyroAngle() {
    double angle = mNavX.getAngle() - getAdjustmentAngle();
    angle %= 360;
    if (angle < 0) {
      angle += 360;
    }

    return angle;
    // or subtract from 360
  }

  public void saveAllZeroOffsets() {
    for (int i = 0; i < 4; i++) {
      mSwerveModules[i].saveZeroOffset();
    }
  }

  public void holonomicDrive(double forward, double strafe, double rotation, boolean fieldOriented) {
    // for (int i = 0; i < 4; i++) {
    // mSwerveModules[i].testDriveMotor(0.1);
    // mSwerveModules[i].testRotationMotor(0.05);

    // }
    // return;

    forward *= getSpeedMultiplier();
    strafe *= getSpeedMultiplier();

    if (fieldOriented) {
      double angleRad = Math.toRadians(getGyroAngle());
      double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
      strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
      forward = temp;
    }

    double a = strafe - rotation * (WHEELBASE / TRACKWIDTH);
    double b = strafe + rotation * (WHEELBASE / TRACKWIDTH);
    double c = forward - rotation * (TRACKWIDTH / WHEELBASE);
    double d = forward + rotation * (TRACKWIDTH / WHEELBASE);

    double[] angles = new double[] { Math.atan2(b, c) * 180 / Math.PI, Math.atan2(b, d) * 180 / Math.PI,
        Math.atan2(a, d) * 180 / Math.PI, Math.atan2(a, c) * 180 / Math.PI };

    double[] speeds = new double[] { Math.sqrt(b * b + c * c), Math.sqrt(b * b + d * d), Math.sqrt(a * a + d * d),
        Math.sqrt(a * a + c * c) };

    double max = speeds[0];

    for (double speed : speeds) {
      if (speed > max) {
        max = speed;
      }
    }

    for (int i = 0; i < 4; i++) { // ddebug
      if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05) {
        mSwerveModules[i].setTargetAngle(angles[i] + 180);
        SmartDashboard.putNumber("Sending Angle" + mSwerveModules[i].getModuleNumber(), angles[i] + 180);
      } else {
        mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle());
        SmartDashboard.putNumber("Sending Angle" + mSwerveModules[i].getModuleNumber(), -999);

      }
      if (enableDrive) {
        mSwerveModules[i].setTargetSpeed(speeds[i]);
      }
    }
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

  public double getMaxAcceleration() {
    return 5.5;
  }

  public double getMaxVelocity() {
    return 10;
  }
}
