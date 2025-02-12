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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Drive;
import frc.robot.Subsystems.DriveSubsystem;

@Logged(name = "Container")
public class RobotContainer {
  public ChassisSpeeds swerveChassis;
  public static Pigeon2 gyro;
  public static PIDController pid;
  public static DriveSubsystem driver;
  public GenericHID j1;
  
    public RobotContainer() {
      gyro = new Pigeon2(Constants.PIGEON_ID);
      pid = new PIDController(Constants.KP_Swerve_ANGLE, Constants.KI_Swerve_ANGLE, Constants.KD_Swerve_ANGLE);
      driver = new DriveSubsystem();
      j1 = new GenericHID(Constants.JOY_PORT);
      configureBindings();

      driver.setDefaultCommand(
        new Drive(
            () -> j1.getRawAxis(1),
            () -> j1.getRawAxis(0),
            () -> j1.getRawAxis(4)*0.6 ,
            () -> j1.getRawButton(0)));
      
    }
  
    private void configureBindings() {}
  
    public static Rotation2d getGyroAngleAsR2D(){
      return new Rotation2d(gyro.getYaw().getValue());
    }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
