package frc.robot.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Constants {

    public static final double kTrackWidth = 0.632;
    public static final double kWheelBase = 0.632;
    public static final double kWheelDiameterMeters = 0.1016; // Diâmetro da roda em metros (4 polegadas convertidas
                                                              // para metros)
    public static final double kWheelCircuferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDriveMotorGearRatio = 6.75; // Relação de transmissão do motor de direção
    public static final double kTurningMotorGearRatio = 21.43; // Relação de transmissão do motor de rotação

    public enum SwerveModulesContants {
        MOD0(// Front Left (Inverted)
                MOTOR_LEFT_ANGULAR_FRONT,
                MOTOR_LEFT_DRIVER_FRONT,
                CANCODER_FRONT_LEFT,
                2),

        MOD1(// Front Right (Inverted)
                MOTOR_RIGHT_ANGULAR_FRONT,
                MOTOR_RIGHT_DRIVER_FRONT,
                CANCODER_FRONT_RIGHT,
                3),

        MOD2(// Back Left (Inverted)
                MOTOR_LEFT_ANGULAR_BACK,
                MOTOR_LEFT_DRIVER_BACK,
                CANCODER_BACK_LEFT,
                4),

        MOD3(// Back Right (Inverted)
                MOTOR_RIGHT_ANGULAR_BACK,
                MOTOR_RIGHT_DRIVER_BACK,
                CANCODER_BACK_RIGHT,
                1);

        public int getModuleNumber() {
            return moduleNumber;
        }

        public int getDriveMotorID() {
            return driveMotorID;
        }

        public int getAngleMotorID() {
            return angleMotorID;
        }

        public int getCancoderID() {
            return cancoderID;
        }

        private int moduleNumber;
        private int driveMotorID;
        private int angleMotorID;
        private int cancoderID;

        SwerveModulesContants(int motorAngular, int motorDriver, int cancoder, int moduleNumber) {
            // TODO Auto-generated constructor stub
            this.angleMotorID = motorAngular;
            this.driveMotorID = motorDriver;
            this.cancoderID = cancoder;
            this.moduleNumber = moduleNumber;
        }

    }

    // Drivetrain Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Frente esquerda
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Frente direita
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Traseira esquerda
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Traseira direita
    );

    // Motor & Encoder IDs
    //
    //FRONTAL LEFT
    public static final int MOTOR_LEFT_DRIVER_FRONT = 1;
    public static final int MOTOR_LEFT_ANGULAR_FRONT = 2;
    public static final int CANCODER_FRONT_LEFT = 11;

    //BACK LEFT
    public static final int MOTOR_LEFT_DRIVER_BACK = 3;
    public static final int MOTOR_LEFT_ANGULAR_BACK = 4;
    public static final int CANCODER_BACK_LEFT = 12;

    //FRONTAL RIGHT
    public static final int MOTOR_RIGHT_DRIVER_BACK = 5;
    public static final int MOTOR_RIGHT_ANGULAR_BACK = 6;
    public static final int CANCODER_FRONT_RIGHT = 13;

    //BACK RIGHT
    public static final int MOTOR_RIGHT_DRIVER_FRONT = 7;
    public static final int MOTOR_RIGHT_ANGULAR_FRONT = 8;
    public static final int CANCODER_BACK_RIGHT = 14;

    // Joystick & Pigeon Configuration
    public static final int DRIVEJOY_PORT = 0;
    public static final int SCOREJOY_PORT = 0;
    public static final int PIGEON_ID = 15;

    // Maximum Speed
    public static final double MAX_SPEED = 1;
    public static final double MAX_POWER = 0.3;
    public static final double MAX_ANGULAR_SPEED = 1;

    //DEADBANDS
    public static final double JOY_DEADBAND = 0.05;


    // PID Constants
    // PID Constants for Swerve
    public static final double KP_Swerve_ANGLE = 0.005;
    public static final double KI_Swerve_ANGLE = 0.0000001;
    public static final double KD_Swerve_ANGLE = 0.0001;

    public static final double[] PIDSwerve = { KP_Swerve_ANGLE, KI_Swerve_ANGLE, KD_Swerve_ANGLE };

    //Intake Subsystem IDs
    public static final int intakeAngularMotor = 0;
    public static final int intakeAlgaeLeft = 0;
    public static final int intakeAlgaeRight = 0;
    public static final int intakeCoral = 0;

    //Intake PID Constants
    public static final double intakeKP = 0.01;
    public static final double intakeKI = 0;
    public static final double intakeKD = 0;

    //INTAKE values
    public static final double coralIntakePower = 0.75;
    public static final double algaeIntakePower = 0.50;

    //Elevator Subsystem IDs
    public static final int leftElevatorMotor = 0;
    public static final int rightElevatorMotor = 0;
    public static final int microSwitchPWMPort = 0;

    //Elevator PID Constants
    public static final double elevatorKP = 0.01;
    public static final double elevatorKI = 0;
    public static final double elevatorKD = 0;

    //Elevator values
    public static final double elevatorSpeedClamper = 0.75;
    public static final double MAXHEIGHT = 0;

    //Alerts
    public static final Alert elevatorMaxHeight = new Alert("Position too height", AlertType.kWarning);

    //Autonomous AngleAligner
    public static final double angleAlignKP = 0.01;
    public static final double angleAlignKI = 0;
    public static final double angleAlignKD = 0;
}
