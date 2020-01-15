/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveModule extends PIDSubsystem{

  public double d_kP, d_kI, d_kD, d_kIz, d_kFF, d_kMaxOutput, d_kMinOutput, d_kToleranceVolts;
  public double a_kP, a_kI, a_kD, a_kIz, a_kFF, a_kMaxOutput, a_kMinOutput, a_kToleranceVolts;

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

  public AnalogInput mEncoder;

  
  /**
   * Creates a new SwerveModule.
   */
  public SwerveModule(int moduleNumber, int angleMotorID, int driveMotorID, int encoderID) {
    super(new PIDController(0.5, 0.0, 0.02));

    mModuleNumber = moduleNumber;
    getZeroOffset();
    mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mEncoder = new AnalogInput(encoderID);
    pidDrive = mDriveMotor.getPIDController();
    angleMotorPIDController();

  }

  @Override
  public void periodic() {
    super.periodic();
    // This method will be called once per scheduler run
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    
  }

  @Override
  public double getMeasurement() {
    // Retyrn the process variable measurment here
    return 0;
  }

  public void angleMotorPIDController() {
    
   
    // a_kIz = 0.01;
    // a_kFF = 0;
    a_kMaxOutput = 0.5;
    a_kMinOutput = -0.5;
    a_kToleranceVolts = 0.01; // 5%
    // pidAngle = new PIDController(a_kP, a_kI, a_kD, mEncoder, this);
    maxVoltage = RobotController.getVoltage5V();
    pidAngle.enableContinuousInput(minVoltage, maxVoltage);
    // pidAngle.setInputRange(minVoltage, maxVoltage); // ddebug set to min and max
    pidAngle.setTolerance(a_kToleranceVolts);
    // pidAngle.setAbsoluteTolerance(a_kToleranceVolts);
    // pidAngle.setContinuous(true);
    // pidAngle.setOutputRange(a_kMinOutput, a_kMaxOutput);

    SmartDashboard.putNumber("P - angle", a_kP);
    SmartDashboard.putNumber("I - angle", a_kI);
    SmartDashboard.putNumber("D - angle", a_kD);
    SmartDashboard.putNumber("I - angle", a_kIz);
    SmartDashboard.putNumber("Max Angle Output", a_kMaxOutput);
    SmartDashboard.putNumber("Min Angle Output", a_kMinOutput);
    // SmartDashboard.putNumber("Set Angle Inches", 0);
    // pidAngle.enable();
    pidAngle.setSetpoint(0);
  }

  public void getZeroOffset() {
    String key = String.format("ZeroOffset%d", getModuleNumber());
    mZeroOffset = Preferences.getInstance().getDouble(key, 0);
    SmartDashboard.putNumber(key, mZeroOffset);
  }

  public int getModuleNumber() {
    return mModuleNumber;
  }



}
