package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Auto.DriveAuto.DriveForwardAuto;
import frc.robot.Commands.TeleopSwerve.Drive;
import frc.robot.ScoreCommand.ElevatorJS;
import frc.robot.Subsystems.ScoreSystem.ElevatorSubsystem;
import frc.robot.Subsystems.ScoreSystem.IntakeSubsystem;
import frc.robot.Subsystems.Sensors.IMUSubsystem;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.Constants;

public class RobotContainer {
    public ChassisSpeeds swerveChassis;
    public static DriveSubsystem driver;
    public static IMUSubsystem imu;
    public static ElevatorSubsystem elevatorEneable;
    public static IntakeSubsystem intake;
    public static PowerDistribution pdh;
    private PS5Controller ps1, ps2;
    private final DriveForwardAuto autoCommand;

    public RobotContainer() {
        imu = new IMUSubsystem();
        driver = new DriveSubsystem();
        elevatorEneable = new ElevatorSubsystem();
        intake = new IntakeSubsystem();
        ps1 = new PS5Controller(Constants.kDriveControllerID);
        ps2 = new PS5Controller(Constants.kScoreControllerID);
        autoCommand = new DriveForwardAuto();
        configureBindings();

        driver.setDefaultCommand(
                new Drive(
                        () -> -ps1.getLeftX(),
                        () -> -ps1.getLeftY(),
                        () -> ps1.getRightX() * 0.6,
                        () -> imu.getIMUAvaliable()));
    }

    private void configureBindings() {
        new JoystickButton(ps1, 4).whileTrue(Commands.run(() -> imu.resetYaw()));

        // Elevator
        new Trigger(() -> ps2.getL1Button())
        .whileTrue(new ElevatorJS(() -> true, () -> false));

        new Trigger(() -> ps2.getR1Button())
        .whileTrue(new ElevatorJS(() -> false, () -> true));

        // intake coral
        new Trigger(() -> ps2.getR2Button())
        .whileTrue(Commands.run(() -> intake.coralIntake(0.75)))
        .whileFalse(Commands.run(() -> intake.coralDisable()));

        new Trigger(() -> ps2.getL2Button())
        .whileTrue(Commands.run(() -> intake.coralIntake(-0.75)))
        .whileFalse(Commands.run(() -> intake.coralDisable()));

        new Trigger(() -> Math.abs(ps2.getLeftY()) > 0)
        .whileTrue(Commands.run(() -> intake.controlCoralAngulationWithAnalog(ps2.getLeftY())))
        .whileFalse(Commands.run(()-> intake.angulationCoralSetPower(0)));

        //SET ELEVATOR HEIGHTS
        //SET L1 HEIGHT
        new Trigger((() -> ps2.getCrossButton()))
        .whileTrue(Commands.run(() -> elevatorEneable.setLPos(0)));
        //SET L2 HEIGHT
        new Trigger((() -> ps2.getSquareButton()))
        .whileTrue(Commands.run(() -> elevatorEneable.setLPos(1)));
        //SET L3 HEIGHT 
        new Trigger((() -> ps2.getCircleButton()))
        .whileTrue(Commands.run(() -> elevatorEneable.setLPos(2)));
        //SET COLLECT POINT
        new Trigger((() -> ps2.getTriangleButton()))
        .whileTrue(Commands.run(() -> elevatorEneable.setLPos(3))); 

        
    }

    public static Rotation2d getGyroAngleAsR2D() {
        return new Rotation2d(imu.getYaw());
    }

    public Command getAutonomousCommand() {
        return autoCommand;
    }
}