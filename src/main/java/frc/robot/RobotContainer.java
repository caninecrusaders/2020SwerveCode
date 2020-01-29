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
import frc.robot.commands.CmdXboxHolonomic;
import frc.robot.commands.CmdZeroYaw;
import frc.robot.input.JoystickX3D;
import frc.robot.input.XboxController;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public JoystickX3D joystickDriver;
  public XboxController xboxDriver;
  public AHRS ahrs;

  private final SwerveDriveSubsystem swerveDriveSubsystem;
  
  private final CmdHolonomicDrive mCmdHolonomicDrive;
  private final CmdXboxHolonomic mCmdXboxHolonomic;
  
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  
  public JoystickX3D getJoystick() {
    return joystickDriver;
  }
  
  public XboxController getXboxDriver() {
    return xboxDriver;
  }
  
  public SwerveDriveSubsystem getSwerveDriveSubsystem() {
    return swerveDriveSubsystem;
  }
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    joystickDriver = new JoystickX3D(0);
    xboxDriver = new XboxController(1);
  
    swerveDriveSubsystem = new SwerveDriveSubsystem();

    mCmdHolonomicDrive = new CmdHolonomicDrive(swerveDriveSubsystem, joystickDriver);
    mCmdXboxHolonomic = new CmdXboxHolonomic(swerveDriveSubsystem, xboxDriver);
    
    swerveDriveSubsystem.setDefaultCommand(mCmdXboxHolonomic);

    
    // Configure the button bindings
    configureButtonBindings();
  }
  
  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    CmdZeroYaw mCmdZeroYaw = new CmdZeroYaw();
  
    joystickDriver.get5Button().whenPressed(mCmdZeroYaw);
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
