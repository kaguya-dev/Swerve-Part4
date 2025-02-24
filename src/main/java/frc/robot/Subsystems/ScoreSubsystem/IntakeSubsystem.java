// Necessary imports for the code to function
package frc.robot.Subsystems.ScoreSubsystem;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

/**
 * Subsystem responsible for controlling the intake mechanism.
 * This subsystem manages the angulation of the intake, as well as the algae and coral intake motors.
 * It uses PID control for precise angulation positioning.
 */
public class IntakeSubsystem extends SubsystemBase {

    // Motors for the intake mechanism
    private SparkMax angulationMotor;
    private SparkMax algaeIntakeLeft; 
    private SparkMax algaeIntakeRight; 
    private SparkMax coralIntake; 

    // PID Controller for angulation positioning
    private PIDController intakePIDController;

    // Encoder to track the angulation position
    private RelativeEncoder angulationEncoder;

    // Configuration objects for the SparkMax motors
    private SparkMaxConfig angulationMotorConfig;
    private SparkMaxConfig algaeLeftMotorConfig;
    private SparkMaxConfig algaeRightMotorConfig;
    private SparkMaxConfig coralMotorConfig;

    /**
     * Constructor for the IntakeSubsystem.
     * Initializes motors, encoders, PID controller, and motor configurations.
     */
    public IntakeSubsystem() {
        // Initialize motors with their respective IDs and motor types
        angulationMotor = new SparkMax(Constants.kIntakeAngularMotorID, MotorType.kBrushed);
        algaeIntakeLeft = new SparkMax(Constants.kIntakeAlgaeLeftID, MotorType.kBrushless);
        algaeIntakeRight = new SparkMax(Constants.kIntakeAlgaeRightID, MotorType.kBrushless);
        coralIntake = new SparkMax(Constants.kIntakeCoralID, MotorType.kBrushless);

        // Initialize the PID controller with constants for intake angulation
        intakePIDController = new PIDController(Constants.kIntakeKP, Constants.kIntakeKI, Constants.kIntakeKD);
        intakePIDController.enableContinuousInput(0, 0.02); 

        // Get the encoder from the angulation motor
        angulationEncoder = angulationMotor.getEncoder();

        // Configure the angulation motor
        angulationMotorConfig = new SparkMaxConfig();
        angulationMotorConfig.idleMode(IdleMode.kBrake); 
        angulationMotor.configure(angulationMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure the left algae intake motor
        algaeLeftMotorConfig = new SparkMaxConfig();
        algaeLeftMotorConfig.inverted(false); 
        algaeLeftMotorConfig.idleMode(IdleMode.kBrake); 

        // Configure the right algae intake motor
        algaeRightMotorConfig = new SparkMaxConfig();
        algaeRightMotorConfig.inverted(true); 
        algaeRightMotorConfig.idleMode(IdleMode.kBrake); 

        // Apply configurations to the algae intake motors
        algaeIntakeLeft.configure(algaeLeftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        algaeIntakeRight.configure(algaeRightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure the coral intake motor
        coralMotorConfig = new SparkMaxConfig();
        coralMotorConfig.idleMode(IdleMode.kBrake); 

        // Apply configuration to the coral intake motor
        coralIntake.configure(coralMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the angulation of the intake to a specific position using the PID controller.
     *
     * @param position The target position in degrees.
     */
    public void setAngularPosition(double position) {
        // Calculate the power using the PID controller
        double power = intakePIDController.calculate(angulationEncoder.getPosition(), position);
        angulationSetPower(power); 
    }

    /**
     * Sets the power of the angulation motor.
     *
     * @param power The power to apply to the angulation motor.
     */
    private void angulationSetPower(double power) {
        angulationMotor.set(power);
    }

    /**
     * Disables the algae intake motors by setting their power to 0.
     */
    public void algaeIntakeDisable() {
        algaeIntakeLeft.set(0);
        algaeIntakeRight.set(0);
    }

    /**
     * Controls the algae intake motors with a specified power and direction.
     *
     * @param power The power to apply to the motors.
     * @param input The direction of the intake (true for one direction, false for the opposite).
     */
    public void algaeIntake(double power, boolean input) {
        if (input) {
            algaeIntakeLeft.set(power);
            algaeIntakeRight.set(-power);
        } else {
            algaeIntakeLeft.set(-power);
            algaeIntakeRight.set(power);
        }
    }

    /**
     * Controls the coral intake motor with a specified power.
     *
     * @param power The power to apply to the coral intake motor.
     */
    public void coralIntake(double power) {
        coralIntake.set(power);
    }

    /**
     * Disables the coral intake motor by setting its power to 0.
     */
    public void coralDisable() {
        coralIntake.set(0);
    }
}