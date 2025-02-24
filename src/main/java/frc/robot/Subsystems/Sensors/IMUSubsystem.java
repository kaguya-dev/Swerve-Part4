// Necessary imports for the code to function
package frc.robot.Subsystems.Sensors;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.Constants;

/**
 * Subsystem responsible for managing the IMU (Inertial Measurement Unit) sensor.
 * This subsystem provides access to the yaw, pitch, and roll angles measured by the IMU.
 */
public class IMUSubsystem extends SubsystemBase {

    // IMU sensor instance
    private Pigeon2 imuSensor;

    // Angles measured by the IMU
    private Angle yawAngle;
    private Angle pitchAngle;
    private Angle rollAngle;

    /**
     * Constructor for the IMUSubsystem.
     * Initializes the IMU sensor and updates the angle values.
     */
    public IMUSubsystem() {
        // Initialize the IMU sensor with the ID from Constants
        imuSensor = new Pigeon2(Constants.kPigeonID);

        // Update the IMU angle values
        updateIMUValues();
    }

    /**
     * Updates the yaw, pitch, and roll angles from the IMU sensor.
     */
    private void updateIMUValues() {
        yawAngle = imuSensor.getYaw().getValue();
        pitchAngle = imuSensor.getPitch().getValue();
        rollAngle = imuSensor.getRoll().getValue();
    }

    /**
     * Resets the yaw angle to 0 degrees.
     */
    public void resetYaw() {
        imuSensor.setYaw(0);
    }

    /**
     * Periodic method called every scheduler cycle.
     * Updates the IMU angle values.
     */
    @Override
    public void periodic() {
        updateIMUValues();
    }

    /**
     * Returns the current yaw angle.
     *
     * @return The yaw angle as an Angle object.
     */
    public Angle getYaw() {
        return yawAngle;
    }

    /**
     * Returns the current pitch angle.
     *
     * @return The pitch angle as an Angle object.
     */
    public Angle getPitch() {
        return pitchAngle;
    }

    /**
     * Returns the current roll angle.
     *
     * @return The roll angle as an Angle object.
     */
    public Angle getRoll() {
        return rollAngle;
    }

    /**
     * Checks if the IMU sensor is connected and functioning.
     *
     * @return True if the IMU is connected, false otherwise.
     */
    public boolean isIMUFound() {
        return imuSensor.isConnected();
    }
}