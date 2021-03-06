/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.Encoder;

public class SwerveModule extends PIDSubsystem{

  public double d_kP, d_kI, d_kD, d_kIz, d_kFF, d_kMaxOutput, d_kMinOutput, d_kToleranceVolts, d_kVelocityTolerance;
  public double a_kP, a_kI, a_kD, a_kIz, a_kFF, a_kMaxOutput, a_kMinOutput, a_kToleranceVolts, a_kVelocityTolerance;

  public PIDController pidAngle;
  public CANPIDController pidDrive;

  private final CANSparkMax mAngleMotor;
  private final CANSparkMax mDriveMotor;
  
  public static boolean enableAngle = true;
  public static final double encoderVolt = 0;

  private final int mModuleNumber;
  private double mZeroOffset;
  private double driveGearRatio = 4;
  private double driveWheelRadius = 2; // find right numbers
  private boolean angleMotorJam = false;
  private long mStallTimeBegin = Long.MAX_VALUE;

  private boolean driveInverted = false;
  private double mLastError = 0;
  private double lastTargetAngle = 0;
  double angleSpeed;
  double minVoltage = 0.0;
  double maxVoltage = 4.8;
  double maxOut = 0;

  double driveHardAmpLimit = 50;
  int driveSoftAmpLimit = 40;

  double angleHardAmpLimit = 40;
  int angleSoftAmpLimit = 30;

  public AnalogInput mEncoder;
  // private final Encoder mEncoder = new Encoder(channelA, channelB)

  
  /**
   * Creates a new SwerveModule.
   */
  public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int encoderID) {
    
    super(new PIDController(0.5, 0.0, 0.02));
    m_enabled = true;
    mModuleNumber = moduleNumber;
    getZeroOffset();
    mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mEncoder = new AnalogInput(encoderID);
    pidAngle = this.getController();
    pidDrive = mDriveMotor.getPIDController();
    driveMotorPIDController();
    angleMotorPIDController();
  }

  @Override
  public void periodic() {
    // super.periodic();
    if (m_enabled) {
      double currentPosition = m_controller.calculate(getMeasurement(), pidAngle.getSetpoint());
      useOutput(currentPosition, pidAngle.getSetpoint());
      // SmartDashboard.putNumber("Current Position" + mModuleNumber, currentPosition);
    }
    // SmartDashboard.putNumber("SetPoint" + mModuleNumber, pidAngle.getSetpoint());
  }

  @Override
  public double getMeasurement() {
    
    // Return the process variable measurment here
    return mEncoder.getAverageVoltage();
  }

  public void driveMotorPIDController() {
    SmartDashboard.putNumber("amp out" + mModuleNumber, 0);
    d_kMinOutput = -0.5;
    d_kMaxOutput = 0.5;
    pidDrive.setOutputRange(d_kMinOutput, d_kMaxOutput);
    mDriveMotor.setSmartCurrentLimit(driveSoftAmpLimit);
    mDriveMotor.setSecondaryCurrentLimit(driveHardAmpLimit);
  }

  public void angleMotorPIDController() {
    
    a_kMaxOutput = 0.5;
    a_kMinOutput = -0.5;
    a_kToleranceVolts = 0.01; // 5%
    a_kVelocityTolerance = 0.0;
    maxVoltage = RobotController.getVoltage5V();
    pidAngle.enableContinuousInput(minVoltage, maxVoltage);
    pidAngle.setTolerance(a_kToleranceVolts, a_kVelocityTolerance);
    pidAngle.setSetpoint(0);

    mAngleMotor.setSmartCurrentLimit(angleSoftAmpLimit);
    mAngleMotor.setSecondaryCurrentLimit(angleHardAmpLimit);
  }

  public void getZeroOffset() {
    String key = String.format("ZeroOffset%d", getModuleNumber());
    mZeroOffset = Preferences.getInstance().getDouble(key, 0);
  }

  public int getModuleNumber() {
    return mModuleNumber;
  }

  private double angleToVoltage(double angle) {
    return angle * ((maxVoltage - minVoltage) / 360.0);
  }

  private double voltageToAngle(double voltage) {
    return (voltage * (360.0 / (maxVoltage - minVoltage))) % 360.0;
  }

  public void setDriveInverted(boolean inverted) {
    driveInverted = inverted;
  }


  public void saveZeroOffset() {
    mZeroOffset = voltageToAngle(mEncoder.getAverageVoltage());
    String key = String.format("ZeroOffset%d", getModuleNumber());
    Preferences.getInstance().putDouble(key, mZeroOffset);
    SmartDashboard.putNumber(key, mZeroOffset);
  }

  public void setTargetAngle(double targetAngle) {

    lastTargetAngle = targetAngle;

    SmartDashboard.putNumber("Module Target Angle " + mModuleNumber, targetAngle % 360);
    SmartDashboard.putNumber("EncoderVoltage" + mModuleNumber, mEncoder.getVoltage());

    targetAngle += mZeroOffset; // ddebug
    targetAngle %= 360;

    // double currentAngle = mAngleMotor.getSelectedSensorPosition(0) * (360.0 /
    // 1024.0);
    double currentAngle = voltageToAngle(mEncoder.getAverageVoltage());
    double currentAngleMod = currentAngle % 360;
    if (currentAngleMod < 0)
      currentAngleMod += 360;

    double delta = currentAngleMod - targetAngle;

    if (delta > 180) {
      targetAngle += 360;
    } else if (delta < -180) {
      targetAngle -= 360;
    }

    delta = currentAngleMod - targetAngle;
    if (delta > 90 || delta < -90) {
      if (delta > 90)
        targetAngle += 180;
      else if (delta < -90)
        targetAngle -= 180;
      mDriveMotor.setInverted(false);
    } else {
      mDriveMotor.setInverted(true);
    }

    targetAngle += currentAngle - currentAngleMod;
    // SmartDashboard.putNumber("targetAngle" + mModuleNumber, targetAngle);

    // targetAngle *= 1024.0 / 360.0;
    targetAngle = angleToVoltage(targetAngle);
    // SmartDashboard.putNumber("targetVoltage" + mModuleNumber, targetAngle);

    pidAngle.setSetpoint(targetAngle); // ddebug set back to targetAngle
  }

  public void setTargetSpeed(double speed) {
    if (driveInverted) {
      speed = -speed;
    }
    pidDrive.setReference(speed, ControlType.kDutyCycle);
  }

  public void setDriveGearRatio(double ratio) {
    driveGearRatio = ratio;
  }

  public double getDriveWheelRadius() {
    return driveWheelRadius;
  }

  public void setDriveWheelRadius(double radius) {
    driveWheelRadius = radius;
  }

  public double getTargetAngle() {
    return lastTargetAngle;
  }

  public void resetMotor() {
    angleMotorJam = false;
    mStallTimeBegin = Long.MAX_VALUE;
    // SmartDashboard.putBoolean("Motor Jammed" + mModuleNumber, angleMotorJam);
  }

  public void setMotionConstraints(double maxAcceleration, double maxVelocity) {
    // need to set max acceleration and max velocity in the sparks
  }

  public void testDriveMotor(double speed) {
    mDriveMotor.set(speed);
  }

  public void testRotationMotor(double speed) {
    mAngleMotor.set(speed);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // SmartDashboard.putNumber("Velocity Error" + mModuleNumber, pidAngle.getVelocityError());

    // TODO Auto-generated method stub
    if (!enableAngle) {
      mAngleMotor.set(0);
      return;
    }
    if (!pidAngle.atSetpoint() && enableAngle) {
      mAngleMotor.set(-output);
    } else {
      mAngleMotor.set(0);
    }

    // SmartDashboard.putNumber("Output" + mModuleNumber, output);
    double ampOut = mDriveMotor.getOutputCurrent();
    if(ampOut > maxOut) {
      maxOut = ampOut;
    }
    SmartDashboard.putNumber("amp out" + mModuleNumber, maxOut);
  }



}
