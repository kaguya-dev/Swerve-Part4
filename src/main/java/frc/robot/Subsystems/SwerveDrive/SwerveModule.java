// Necessary imports for the code to function
package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;
import frc.robot.Utils.Constants.SwerveModulesContants;

/**
 * Subsystem representing a single swerve module.
 * This subsystem controls the turn and speed motors, manages the absolute encoder,
 * and implements PID control for precise module angle positioning.
 */
public class SwerveModule extends SubsystemBase {

    // Motors for turning and driving the swerve module
    private SparkMax turnMotor;
    private SparkMax speedMotor;

    // Current state of the swerve module
    private SwerveModuleState moduleState;

    // Absolute encoder for tracking the module's angle
    private CANcoder absoluteEncoder;

    // Relative encoder for tracking the module's speed
    private RelativeEncoder speedEncoder;

    // PID controller for angle positioning
    private PIDController anglePIDController;

    // Flag to indicate if the module is on the right side of the robot
    private boolean isRight;

    // Identifier for the module 
    private int moduleNumber;

    /**
     * Constructor for the SwerveModule.
     *
     * @param moduleID The constants for the swerve module (angle motor ID, drive motor ID, CANcoder ID, etc.).
     */
    public SwerveModule(SwerveModulesContants moduleID) {
        // Initialize motors and encoders
        this.turnMotor = new SparkMax(moduleID.getAngleMotorID(), MotorType.kBrushless);
        this.speedMotor = new SparkMax(moduleID.getDriveMotorID(), MotorType.kBrushless);
        this.absoluteEncoder = new CANcoder(moduleID.getCancoderID());
        this.speedEncoder = speedMotor.getEncoder();

        // Set module number and initialize state
        this.moduleNumber = moduleID.getModuleNumber();
        this.moduleState = new SwerveModuleState();
        updateState();

        // Initialize PID controller for angle control
        this.anglePIDController = new PIDController(Constants.kSwerveAngleKP, Constants.kSwerveAngleKI, Constants.kSwerveAngleKD);
        anglePIDController.setIZone(5); 
        anglePIDController.setTolerance(1); 
    }

    /**
     * Periodic method called every scheduler cycle.
     * Updates the module's state (angle and speed).
     */
    @Override
    public void periodic() {
        updateState();
    }

    /**
     * Updates the module's state (angle and speed) based on encoder readings.
     */
    private void updateState() {
        // Update the module's angle
        moduleState.angle = getAngleInR2D();

        // Calculate the module's speed in meters per second
        double stateSpeed = (speedEncoder.getVelocity() * Constants.kWheelCircumferenceMeters) / 60;
        moduleState.speedMetersPerSecond = stateSpeed;

        SmartDashboard.putNumber("Speed mod " + this.moduleNumber, stateSpeed);
    }

    /**
     * Returns the current state of the swerve module.
     *
     * @return The current SwerveModuleState (angle and speed).
     */
    public SwerveModuleState getState() {
        updateState();
        return moduleState;
    }

    /**
     * Returns the current position of the swerve module.
     *
     * @return The current SwerveModulePosition (distance and angle).
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(speedEncoder.getPosition(), getAngleInR2D());
    }

    /**
     * Returns the current angle of the swerve module as a Rotation2d object.
     *
     * @return The current angle of the module.
     */
    private Rotation2d getAngleInR2D() {
        // Display the module's angle on SmartDashboard
        SmartDashboard.putNumber("Angle mod " + moduleNumber, absoluteEncoder.getAbsolutePosition().getValue().magnitude() * 360);
        return new Rotation2d(absoluteEncoder.getAbsolutePosition().getValue());
    }

    /**
     * Sets the power of the speed motor.
     *
     * @param power The power to apply to the speed motor.
     */
    public void setSpeedPower(double power) {
        speedMotor.set(power);
    }

    /**
     * Sets the power of the turn motor.
     *
     * @param power The power to apply to the turn motor.
     */
    public void setTurnSpeed(double power) {
        turnMotor.set(power);
    }

    /**
     * Sets whether the module is on the right side of the robot.
     *
     * @param right True if the module is on the right side, false otherwise.
     */
    public void isRight(boolean right) {
        if (right) {
            isRight = true;
        } else {
            isRight = false;
        }
    }

    /**
     * Sets the desired state of the swerve module (angle and speed).
     *
     * @param desiredState The desired SwerveModuleState (angle and speed).
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the desired state to minimize rotation
        desiredState.optimize(getState().angle);

        // Calculate the speed power and apply it to the speed motor
        double speedPower = (desiredState.speedMetersPerSecond / Constants.kMaxSpeed) * (isRight ? -1 : 1);
        SmartDashboard.putNumber("SET PRE-Speed mod " + moduleNumber, speedPower);
        setSpeedPower(MathUtil.clamp(speedPower, -Constants.kMaxPower, Constants.kMaxPower));
        SmartDashboard.putNumber("SET CLAMPED-speed mod " + moduleNumber, speedPower);

        // Calculate the setpoint and measure for the PID controller
        double setpoint = desiredState.angle.getDegrees();
        double measure = getState().angle.getDegrees();

        // Adjust the setpoint to handle wrap-around ( 0° and 360° are the same)
        double error = setpoint - measure;
        if (Math.abs(error) > 180) {
            if (error > 0) {
                setpoint -= 360;
            } else {
                setpoint += 360;
            }
        }

        // Calculate the PID output and apply it to the turn motor
        double pidValue = anglePIDController.calculate(measure, setpoint);
        SmartDashboard.putNumber("SET Angle mod " + moduleNumber, pidValue);
        setTurnSpeed(pidValue);
    }

    /**
     * Returns the module's number.
     *
     * @return The module's number 
     */
    public int getModuleNumber() {
        return moduleNumber;
    }
}