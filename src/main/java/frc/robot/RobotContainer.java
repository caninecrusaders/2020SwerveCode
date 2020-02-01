/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.CmdHolonomicDrive;
import frc.robot.commands.CmdTwoJoystickHolonomic;
import frc.robot.commands.CmdXboxHolonomic;
import frc.robot.commands.CmdZeroYaw;
import frc.robot.input.JoystickX3D;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  
  public JoystickX3D joystickDriverOne;
  public JoystickX3D joystickDriverTwo;
  public XboxController xboxDriverOne;
  public XboxController xboxDriverTwo;
  public AHRS ahrs;

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final CmdHolonomicDrive mCmdHolonomicDrive;
  private final CmdXboxHolonomic mCmdXboxHolonomic;
  private final CmdTwoJoystickHolonomic mCmdTwoJoystickHolonomic;
  
  public JoystickX3D getJoystickOne() {
    return joystickDriverOne;
  }

  public JoystickX3D getJoystickTwo() {
    return joystickDriverTwo;
  }
  
  public XboxController getXboxDriverOne() {
    return xboxDriverOne;
  }

  public XboxController getXboxDriverTwo() {
    return xboxDriverTwo;
  }
  
  public SwerveDriveSubsystem getSwerveDriveSubsystem() {
    return swerveDriveSubsystem;
  }
  
  public RobotContainer() {
    joystickDriverOne = new JoystickX3D(0);
    joystickDriverTwo = new JoystickX3D(1);
    xboxDriverOne = new XboxController(2);
    xboxDriverTwo = new XboxController(3);
  
    swerveDriveSubsystem = new SwerveDriveSubsystem();

    mCmdHolonomicDrive = new CmdHolonomicDrive(swerveDriveSubsystem, joystickDriverOne);
    mCmdXboxHolonomic = new CmdXboxHolonomic(swerveDriveSubsystem, xboxDriverOne);
    mCmdTwoJoystickHolonomic = new CmdTwoJoystickHolonomic(swerveDriveSubsystem, joystickDriverOne, joystickDriverTwo);
    
    
    // swerveDriveSubsystem.setDefaultCommand(mCmdXboxHolonomic);
    swerveDriveSubsystem.setDefaultCommand(mCmdHolonomicDrive);
    // swerveDriveSubsystem.setDefaultCommand(mCmdTwoJoystickHolonomic);
    
    configureButtonBindings();
  }
  
  private void configureButtonBindings() {
    CmdZeroYaw mCmdZeroYaw = new CmdZeroYaw();
  
    joystickDriverOne.get5Button().whenPressed(mCmdZeroYaw);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_autoCommand;
  // }
}
