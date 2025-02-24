// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Commands.TeleopSwerve.Drive;
import frc.robot.Subsystems.ScoreSubsystem.IntakeSubsystem;
import frc.robot.Subsystems.Sensors.FishEye;
import frc.robot.Subsystems.Sensors.IMUSubsystem;
import frc.robot.Subsystems.Sensors.LimelightSubsystem;
import frc.robot.Utils.Constants;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public ChassisSpeeds swerveChassis;
  public static DriveSubsystem driver;
  public static IMUSubsystem imu;
  public static IntakeSubsystem scoreIntake;
  public static LimelightSubsystem limelight;
  public static FishEye fisheye;
  public GenericHID j1, j2;
  public PS5Controller ps1, ps2;

  public RobotContainer() {
    imu = new IMUSubsystem();
    driver = new DriveSubsystem();
    scoreIntake = new IntakeSubsystem();
    limelight = new LimelightSubsystem();
    fisheye = new FishEye();
    j1 = new GenericHID(Constants.DRIVEJOY_PORT);
    j2 = new GenericHID(Constants.SCOREJOY_PORT);
    configureBindings();

    driver.setDefaultCommand(
        new Drive(
            () -> j1.getRawAxis(1),
            () -> -j1.getRawAxis(0),
            () -> j1.getRawAxis(4) * 0.6,
            () -> imu.isIMUFound()));

    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
     autoChooser = AutoBuilder.buildAutoChooser("hahaha");

    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private void configureBindings() {

    new JoystickButton(j1, 2).whileTrue(Commands.run(() -> imu.resetYaw()));

    new JoystickButton(j2, 3)
        .onTrue(Commands.run(() -> scoreIntake.coralIntake(Constants.coralIntakePower)))
        .onFalse(Commands.run(() -> scoreIntake.coralDisable()));

    new JoystickButton(j2, 2)
        .onTrue(Commands.run(() -> scoreIntake.algaeIntake(Constants.algaeIntakePower, true)))
        .onFalse(Commands.run(() -> scoreIntake.algaeIntakeDisable()));

    new JoystickButton(j2, 1)
        .onTrue(Commands.run(() -> scoreIntake.algaeIntake(Constants.algaeIntakePower, false)))
        .onFalse(Commands.run(() -> scoreIntake.algaeIntakeDisable()));
  }

  public static Rotation2d getGyroAngleAsR2D() {
    return new Rotation2d(imu.getYaw());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
