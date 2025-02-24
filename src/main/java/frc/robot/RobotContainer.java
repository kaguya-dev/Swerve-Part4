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
import frc.robot.Subsystems.Sensors.IMUSubsystem;
import frc.robot.Subsystems.Sensors.LimelightSubsystem;
import frc.robot.Utils.Constants;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {

    // Auto chooser for selecting autonomous routines
    private final SendableChooser<Command> autoChooser;

    // Chassis speeds for swerve drive
    public ChassisSpeeds swerveChassis;

    // Subsystems
    public static DriveSubsystem driver; 
    public static IMUSubsystem imu; 
    public static IntakeSubsystem scoreIntake; 
    public static LimelightSubsystem limelight; 

    // Input devices
    public GenericHID j1, j2; 
    public PS5Controller ps1, ps2; 

    /**
     * Constructor for the RobotContainer.
     * Initializes subsystems, input devices, and configures default commands and bindings.
     */
    public RobotContainer() {
        // Initialize subsystems
        imu = new IMUSubsystem();
        driver = new DriveSubsystem();
        scoreIntake = new IntakeSubsystem();
        limelight = new LimelightSubsystem();

        // Initialize input devices
        j1 = new GenericHID(Constants.kDriveControllerID);
        j2 = new GenericHID(Constants.kScoreControllerID);

        // Configure button bindings
        configureBindings();

        // Set the default command for the drive subsystem
        driver.setDefaultCommand(
                new Drive(
                        () -> j1.getRawAxis(1), // X-axis input
                        () -> -j1.getRawAxis(0), // Y-axis input
                        () -> j1.getRawAxis(4) * 0.6, // Rotation input 
                        () -> imu.isIMUFound() // Field-centric mode 
                )
        );

        // Build an auto chooser for selecting autonomous routines
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Configures button bindings for the robot.
     * Maps buttons to specific commands.
     */
    private void configureBindings() {
        // Reset the IMU yaw when button 2 on joystick 1 is pressed
        new JoystickButton(j1, 2).whileTrue(Commands.run(() -> imu.resetYaw()));

        // Control coral intake with button 3 on joystick 2
        new JoystickButton(j2, 3)
                .onTrue(Commands.run(() -> scoreIntake.coralIntake(Constants.coralIntakePower))) // Enable coral intake
                .onFalse(Commands.run(() -> scoreIntake.coralDisable())); // Disable coral intake

        // Control algae intake (forward) with button 2 on joystick 2
        new JoystickButton(j2, 2)
                .onTrue(Commands.run(() -> scoreIntake.algaeIntake(Constants.algaeIntakePower, true))) // Enable algae intake (forward)
                .onFalse(Commands.run(() -> scoreIntake.algaeIntakeDisable())); // Disable algae intake

        // Control algae intake (reverse) with button 1 on joystick 2
        new JoystickButton(j2, 1)
                .onTrue(Commands.run(() -> scoreIntake.algaeIntake(Constants.algaeIntakePower, false))) // Enable algae intake (reverse)
                .onFalse(Commands.run(() -> scoreIntake.algaeIntakeDisable())); // Disable algae intake
    }

    /**
     * Returns the current gyro angle as a Rotation2d object.
     *
     * @return The current gyro angle.
     */
    public static Rotation2d getGyroAngleAsR2D() {
        return new Rotation2d(imu.getYaw());
    }

    /**
     * Returns the selected autonomous command from the auto chooser.
     *
     * @return The selected autonomous command.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}