package frc.robot.Utils;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class TunerConstants {
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    private static final Current kSlipCurrent = Amps.of(120.0);

    private static final SparkMaxConfig driveInitialConfigs = new SparkMaxConfig();
    private static final SparkMaxConfig steerInitialConfigs = new SparkMaxConfig();

   
        /* .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );*/

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    private static final Pigeon2Configuration pigeonConfigs = null;

    public static final CANBus kCANBus = new CANBus("canivore", "./logs/example.hoot");

    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    private static final double kCoupleRatio = 3.8181818181818183;
    private static final double kDriveGearRatio = 7.363636363636365;
    private static final double kSteerGearRatio = 15.42857142857143;
    private static final Distance kWheelRadius = Inches.of(2.167);

    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;

    private static final int kPigeonId = 1;

    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
            .withCANBusName(kCANBus.getName())
            .withPigeon2Id(kPigeonId)
            .withPigeon2Configs(pigeonConfigs);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            //.withDriveMotorType(kDriveMotorType)
            //.withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
           // .withDriveMotorInitialConfigs(driveInitialConfigs)
           // .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 2;
    private static final int kFrontLeftEncoderId = 1;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.15234375);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final Distance kFrontLeftXPos = Inches.of(10);
    private static final Distance kFrontLeftYPos = Inches.of(10);

    private static final int kFrontRightDriveMotorId = 1;
    private static final int kFrontRightSteerMotorId = 0;
    private static final int kFrontRightEncoderId = 0;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.4873046875);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;
    private static final Distance kFrontRightXPos = Inches.of(10);
    private static final Distance kFrontRightYPos = Inches.of(-10);

    private static final int kBackLeftDriveMotorId = 7;
    private static final int kBackLeftSteerMotorId = 6;
    private static final int kBackLeftEncoderId = 3;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.219482421875);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;
    private static final Distance kBackLeftXPos = Inches.of(-10);
    private static final Distance kBackLeftYPos = Inches.of(10);

    private static final int kBackRightDriveMotorId = 5;
    private static final int kBackRightSteerMotorId = 4;
    private static final int kBackRightEncoderId = 2;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.17236328125);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;
    private static final Distance kBackRightXPos = Inches.of(-10);
    private static final Distance kBackRightYPos = Inches.of(-10);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency, modules
            );
        }

        public TunerSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules
        ) {
            super(
                TalonFX::new, TalonFX::new, CANcoder::new,
                drivetrainConstants, odometryUpdateFrequency,
                odometryStandardDeviation, visionStandardDeviation, modules
            );
        }
    }
}