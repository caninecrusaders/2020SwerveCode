/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.JoystickX3D;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CmdTwoJoystickHolonomic extends CommandBase {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private JoystickX3D joystickOne;
  private JoystickX3D joystickTwo;
  /**
   * Creates a new CmdTwoJoystickHolonomic.
   */
  public CmdTwoJoystickHolonomic(SwerveDriveSubsystem drivetrain, JoystickX3D joystickOneIn, JoystickX3D joystickTwoIn) {
    swerveDriveSubsystem = drivetrain;
    joystickOne = joystickOneIn;
    joystickTwo = joystickTwoIn;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  private double deadband(double input) {
    if (Math.abs(input) < 0.05)
      return 0;
    return input;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forward = -joystickOne.getYAxis();

    double strafe = joystickOne.getXAxis();

    double rotation = joystickTwo.getZAxis();
    // System.out.printf("F:%f S:%f R:%f", forward, strafe, rotation);

    forward *= Math.abs(forward);
    strafe *= Math.abs(strafe);
    rotation *= Math.abs(rotation);

    forward = deadband(forward);
    strafe = deadband(strafe);
    rotation = deadband(rotation);

    // SmartDashboard.putNumber("Forward", forward);
    // SmartDashboard.putNumber("Strafe", strafe);
    // SmartDashboard.putNumber("Rotation", rotation);

    swerveDriveSubsystem.holonomicDrive(forward, strafe, rotation, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
