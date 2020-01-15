/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.input.JoystickX3D;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Robot;

public class cmdHolonomicDrive extends CommandBase {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private JoystickX3D joystick;
  /**
   * Creates a new cmdHolonomicDrive.
   */
  public cmdHolonomicDrive(SwerveDriveSubsystem driveTrain, JoystickX3D joystickIn) {
    swerveDriveSubsystem = driveTrain;
    joystick = joystickIn;
    addRequirements(driveTrain);
  }

  private double deadband(double input) {
    if (Math.abs(input) < 0.05)
      return 0;
    return input;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forward = -joystick.getYAxis();

    double strafe = joystick.getXAxis();

    double rotation = joystick.getZAxis();
    // System.out.printf("F:%f S:%f R:%f", forward, strafe, rotation);

    forward *= Math.abs(forward);
    strafe *= Math.abs(strafe);
    rotation *= Math.abs(rotation);

    forward = deadband(forward);
    strafe = deadband(strafe);
    rotation = deadband(rotation);

    SmartDashboard.putNumber("Forward", forward);
    SmartDashboard.putNumber("Strafe", strafe);
    SmartDashboard.putNumber("Rotation", rotation);

    swerveDriveSubsystem.holonomicDrive(forward, strafe, rotation);
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
