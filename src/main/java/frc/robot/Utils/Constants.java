// Necessary imports for the code to function
package frc.robot.Utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Utility class containing constants used throughout the robot code.
 * This includes motor IDs, PID constants, physical dimensions, and other configuration values.
 */
public class Constants {

    // Physical dimensions of the robot
    public static final double kTrackWidth = 0.632; // Width between the center of the left and right wheels in meters
    public static final double kWheelBase = 0.632; // Distance between the center of the front and back wheels in meters
    public static final double kWheelDiameterMeters = 0.1016; // Diameter of the wheels in meters or 4"
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // Circumference of the wheels in meters
    public static final double kDriveMotorGearRatio = 6.75; // Gear ratio for the drive motor (L2)
    public static final double kTurningMotorGearRatio = 21.43; // Gear ratio for the turning motor

    /**
     * Enum representing the constants for each swerve module.
     * This includes motor IDs, CANcoder IDs, and module numbers.
     */
    public enum SwerveModuleConstants {
       
            MOD0(// Front Left (Inverted)
                    MOTOR_LEFT_ANGULAR_FRONT,
                    MOTOR_LEFT_DRIVER_FRONT,
                    CANCODER_FRONT_LEFT,
                    CANCODER_FRONT_LEFT_OFFSET,
                    2, true),
    
            MOD1(// Front Right (Inverted)
                    MOTOR_RIGHT_ANGULAR_FRONT,
                    MOTOR_RIGHT_DRIVER_FRONT,
                    CANCODER_FRONT_RIGHT,
                    CANCODER_FRONT_RIGHT_OFFSET,
                    3, false),
    
            MOD2(// Back Left (Inverted)
                    MOTOR_LEFT_ANGULAR_BACK,
                    MOTOR_LEFT_DRIVER_BACK,
                    CANCODER_BACK_LEFT,
                    CANCODER_BACK_LEFT_OFFSET,
                    0, false),
    
            MOD3(// Back Right (Inverted)
                    MOTOR_RIGHT_ANGULAR_BACK,
                    MOTOR_RIGHT_DRIVER_BACK,
                    CANCODER_BACK_RIGHT,
                    CANCODER_BACK_RIGHT_OFFSET,
                    1, false);
                
    
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
    
            public double getCancoderOffSet(){
                return cancoderOffset;
            }
    
            public boolean isReversed() {
                return isReversed;
            }
    
            private int moduleNumber;
            private int driveMotorID;
            private int angleMotorID;
            private int cancoderID;
            private double cancoderOffset;
            private boolean isReversed;
            
    
            SwerveModuleConstants(int motorAngular, int motorDriver, int cancoder, double cancoderOffset, int moduleNumber, boolean isReversed ) {
                this.angleMotorID = motorAngular;
                this.driveMotorID = motorDriver;
                this.cancoderID = cancoder;
                this.cancoderOffset = cancoderOffset;
                this.moduleNumber = moduleNumber;
                this.isReversed = isReversed;
            }
        }
    // Drivetrain Kinematics
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front left
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // Back left
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // Back right
    );

   // Motor & Encoder IDs
    //
    //FRONTAL LEFT
    public static final int MOTOR_LEFT_DRIVER_FRONT = 1;
    public static final int MOTOR_LEFT_ANGULAR_FRONT = 2;
    public static final int CANCODER_FRONT_LEFT = 21;
    public static final double CANCODER_FRONT_LEFT_OFFSET = -0.20849609375;

    //BACK LEFT
    public static final int MOTOR_LEFT_DRIVER_BACK = 3;
    public static final int MOTOR_LEFT_ANGULAR_BACK = 4;
    public static final int CANCODER_BACK_LEFT = 22;
    public static final double CANCODER_BACK_LEFT_OFFSET = -0.006103515625;

    //FRONTAL RIGHT
    public static final int MOTOR_RIGHT_DRIVER_BACK = 5;
    public static final int MOTOR_RIGHT_ANGULAR_BACK = 6;
    public static final int CANCODER_FRONT_RIGHT = 23;
    public static final double CANCODER_FRONT_RIGHT_OFFSET = -0.76171875 ;

    //BACK RIGHT
    public static final int MOTOR_RIGHT_DRIVER_FRONT = 7;
    public static final int MOTOR_RIGHT_ANGULAR_FRONT = 8;
    public static final int CANCODER_BACK_RIGHT = 24;
    public static final double CANCODER_BACK_RIGHT_OFFSET = -0.209228515625;

    // Joystick & Pigeon Configuration
    public static final int kDriveControllerID = 0;
    public static final int kScoreControllerID = 1;
    public static final int kPigeonID = 15;

    // Maximum Speed
    public static final double kMaxSpeed = 1; // Maximum speed in meters per second
    public static final double kMaxPower = 0.3; // Maximum power output
    public static final double kMaxAngularSpeed = 1; // Maximum angular speed in radians per second

    // Deadbands
    public static final double kControllerDeadband = 0.05; // Deadband for controller inputs

    // PID Constants for Swerve
    public static final double kSwerveAngleKP = 0.002; 
    public static final double kSwerveAngleKI = 0.0000001; 
    public static final double kSwerveAngleKD = 0.000;

    // Array of PID constants for swerve
    public static final double[] PIDSwerve = { kSwerveAngleKD, kSwerveAngleKI, kSwerveAngleKD };

    // Intake Subsystem IDs
    public static final int kIntakeAngularAlgaeMotorID = 10; // ID for the intake angular motor
    public static final int kIntakeAngularCoralMotorID = 12; // ID for the intake angular motor
    public static final int kIntakeAlgaeLeftID = 11; // ID for the left algae intake motor
    public static final int kIntakeAlgaeRightID = 9; // ID for the right algae intake motor
    public static final int kIntakeCoralID = 13; // ID for the coral intake motor

    // Intake PID Constants
    public static final double kIntakeKP = 0.01; // Proportional constant for intake control
    public static final double kIntakeKI = 0; // Integral constant for intake control
    public static final double kIntakeKD = 0; // Derivative constant for intake control

    // Intake values
    public static final double coralIntakePower = 0.75; // Power for coral intake
    public static final double algaeIntakePower = 0.50; // Power for algae intake

    // Elevator Subsystem IDs
    public static final int kLeftElevatorMotorID = 14; // ID for the left elevator motor
    public static final int kRightElevatorMotorID = 13; // ID for the right elevator motor
    public static final int kMicroSwitchPWMPort = 0; // PWM port for the microswitch

    // Elevator PID Constants
    public static final double kElevatorKP = 0.01; // Proportional constant for elevator control
    public static final double kElevatorKI = 0; // Integral constant for elevator control
    public static final double kElevatorKD = 0; // Derivative constant for elevator control

    // Elevator values
    public static final double kElevatorSpeedClamper = 0.75; // Speed limit for the elevator
    public static final double kMaxHeight = 0; // Maximum height for the elevator

    // Alerts
    public static final Alert kElevatorMaxHeight = new Alert("Position too high", AlertType.kWarning); // Alert for elevator max height

    // Autonomous Angle Aligner PID Constants
    public static final double kAngleAlignKP = 0.01; // Proportional constant for angle alignment
    public static final double kAngleAlignKI = 0; // Integral constant for angle alignment
    public static final double kAngleAlignKD = 0; // Derivative constant for angle alignment
    public static int intakeCoral;
}