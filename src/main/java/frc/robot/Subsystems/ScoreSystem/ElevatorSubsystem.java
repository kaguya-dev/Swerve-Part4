// Necessary imports for the code to function
package frc.robot.Subsystems.ScoreSystem;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;


/**
 * Subsystem responsible for controlling the elevator mechanism.
 * This subsystem uses two SparkMax motors, a PID controller, and a calibration switch
 * to manage the elevator's position and ensure it operates within safe limits.
 */
public class ElevatorSubsystem extends SubsystemBase {

    // Motors for the elevator
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    // Encoder to track the elevator's position
    private RelativeEncoder elevatorEncoder;

    // Calibration switch to reset the elevator's position
    private DigitalInput calibrationSwitch;

    // Flag to indicate if the elevator has reached its maximum height
    private boolean isAtMaxHeight;

    // Configuration objects for the SparkMax motors
    private SparkMaxConfig leftMotorConfig;
    private SparkMaxConfig rightMotorConfig;

    // PID controller for precise elevator positioning
    private PIDController elevatorPIDController;

    /**
     * Constructor for the ElevatorSubsystem.
     * Initializes motors, encoders, PID controller, and calibration switch.
     */
    public ElevatorSubsystem() {
        // Initialize the PID controller with constants for elevator control
        elevatorPIDController = new PIDController(Constants.kElevatorKP, Constants.kElevatorKI, Constants.kElevatorKD);

        // Initialize the left and right motors
        leftMotor = new SparkMax(Constants.kLeftElevatorMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(Constants.kRightElevatorMotorID, MotorType.kBrushless);

        // Initialize motor configurations
        leftMotorConfig = new SparkMaxConfig();
        rightMotorConfig = new SparkMaxConfig();

        // Configure the left motor
        leftMotorConfig.inverted(false); 
        leftMotorConfig.idleMode(IdleMode.kBrake); 

        // Configure the right motor
        rightMotorConfig.inverted(true); 
        rightMotorConfig.idleMode(IdleMode.kBrake); 

        // Get the encoder from the left motor
        elevatorEncoder = leftMotor.getEncoder();

        // Initialize the calibration switch
        calibrationSwitch = new DigitalInput(Constants.kMicroSwitchPWMPort);

        // Initialize the max height flag
        isAtMaxHeight = false;
        
        leftMotor.configure(leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(rightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Shuffleboard.getTab("ScoreSystem").addBoolean("CalibrationSwitch", () -> calibrationSwitch.get());
        Shuffleboard.getTab("ScoreSystem").addDouble("ElevatorPosition", () -> elevatorEncoder.getPosition());
    }

    /**
     * Sets the power to the elevator motors, clamping the value to ensure safe operation.
     *
     * @param power The power to apply to the motors, clamped between -kElevatorSpeedClamper and kElevatorSpeedClamper.
     */
    private void powerElevator(double power) {
        // Clamp the power to ensure it stays within safe limits
        power = MathUtil.clamp(power, -0.4, 0.4);

        // Apply the power to both motors
        leftMotor.set(power);
        rightMotor.set(power);
    }

    /**
     * Moves the elevator to a specific position using the PID controller.
     *
     * @param position The target position for the elevator.
     */
    public void setElevator(double position) {
        // Calculate the output using the PID controller
        double output = elevatorPIDController.calculate(elevatorEncoder.getPosition(), position);

        // Check if the target position is within the maximum height limit
        if (position < Constants.kMaxHeight) {
            isAtMaxHeight = false; 
            powerElevator(output); 
        } else {
            isAtMaxHeight = true; 
        }
    }

    /**
     * Resets the elevator's position to zero using the calibration switch.
     */
    private void lowCalibratePID() {
        elevatorEncoder.setPosition(0);
    }

    /**
     * Periodic method called every scheduler cycle.
     * Checks the calibration switch and updates the max height flag.
     */
    @Override
    public void periodic() {
        // Check if the calibration switch is triggered
        if (calibrationSwitch.get()) {
            lowCalibratePID(); // Reset the elevator position
        }
    }
}