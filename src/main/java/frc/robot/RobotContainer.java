// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.TeleopSwerve.Drive;
import frc.robot.Subsystems.Sensors.IMUSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.Constants;

@Logged(name = "Container")
public class RobotContainer {
  public ChassisSpeeds swerveChassis;
  public static DriveSubsystem driver;
  public static IMUSubsystem imu;
  
  public GenericHID j1;
  public PS5Controller ps1;
  
    public RobotContainer() {
      imu = new IMUSubsystem();
      driver = new DriveSubsystem();
      //j1 = new GenericHID(Constants.JOY_PORT);
      ps1 = new PS5Controller(Constants.kDriveControllerID);
      configureBindings();
      

      // driver.setDefaultCommand(
      //   new Drive(
      //       () -> j1.getRawAxis(1),
      //       () -> -j1.getRawAxis(0),
      //       () -> j1.getRawAxis(4)*0.8 ,
      //       () -> true));

      driver.setDefaultCommand(
          new Drive(
            () -> ps1.getLeftX(),
            () -> -ps1.getLeftY(),
            () -> ps1.getRightX()*0.6,
            () -> imu.getIMUAvaliable()
          )
      );
      
    }
  
    private void configureBindings() {

      new JoystickButton(ps1,4).whileTrue(Commands.run(() -> imu.resetYaw())); 
    }
  
    public static Rotation2d getGyroAngleAsR2D(){
      return new Rotation2d(imu.getYaw());
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}