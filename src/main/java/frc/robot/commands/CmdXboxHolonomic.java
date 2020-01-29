/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class CmdXboxHolonomic extends CommandBase {
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private XboxController xboxController;
  /**
   * Creates a new CmdXboxHolonomic.
   */
  public CmdXboxHolonomic(SwerveDriveSubsystem driveTrain, XboxController xboxIn) {
    swerveDriveSubsystem = driveTrain;
    xboxController = xboxIn;
    addRequirements(driveTrain);
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
    double forward = xboxController.getLeftYValue();

    double strafe = xboxController.getLeftXValue();

    double rotation = xboxController.getRightXValue();
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
