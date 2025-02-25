// Necessary imports for the code to function
package frc.robot.Subsystems.Sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.LimelightHelpers;

/**
 * Subsystem responsible for interfacing with the Limelight vision system.
 * This subsystem provides methods to retrieve and manage data from the Limelight,
 * such as target position (X, Y) and area, as well as setting the target ID.
 */
public class LimelightSubsystem extends SubsystemBase {

    // NetworkTable instance for accessing Limelight data
    private NetworkTable lime;

    // NetworkTable entries for Limelight values
    private NetworkTableEntry tX;
    private NetworkTableEntry tY;
    private NetworkTableEntry tA;

    // Variables to store the latest Limelight values
    private double tXValue;
    private double tYValue;
    private double tAreaValue;

    /**
     * Constructor for the LimelightSubsystem.
     * Initializes the NetworkTable and its entries for accessing Limelight data.
     */
    public LimelightSubsystem() {
        // Get the Limelight NetworkTable
        lime = NetworkTableInstance.getDefault().getTable("limelight");

        // Initialize NetworkTable entries for target X, Y, and area
        tX = lime.getEntry("tx");
        tY = lime.getEntry("ty");
        tA = lime.getEntry("ta");

        // Initialize variables to store the latest Limelight values
        tXValue = tX.getDouble(0.0);
        tYValue = tY.getDouble(0.0);
        tAreaValue = tA.getDouble(0.0);
    }

    /**
     * Periodic method called every scheduler cycle.
     * Updates the Limelight values (X, Y, and area) from the NetworkTable.
     */
    @Override
    public void periodic() {
        tXValue = tX.getDouble(0.0);
        tYValue = tX.getDouble(0.0);
        tAreaValue = tY.getDouble(0.0);
    }

    /**
     * Sets the priority target ID for the Limelight.
     *
     * @param ID The ID of the target to prioritize.
     */
    public void setTargetID(int ID) {
        LimelightHelpers.setPriorityTagID("limelight", ID);
    }

    /**
     * Checks if the Limelight has detected a valid target.
     *
     * @return True if a target is detected (area > 0), false otherwise.
     */
    public boolean getTargetLime() {
        if (tAreaValue > 0) {
            return true;
        }
        return false;
    }

    /**
     * Returns the latest X value from the Limelight.
     *
     * @return The X value (horizontal offset) of the target.
     */
    public double getLimeXValue() {
        return tXValue;
    }

    /**
     * Returns the latest Y value from the Limelight.
     *
     * @return The Y value (vertical offset) of the target.
     */
    public double getLimeYValue() {
        return tYValue;
    }

    /**
     * Returns the latest area value from the Limelight.
     *
     * @return The area of the target.
     */
    public double getLimeAreaValue() {
        return tAreaValue;
    }
}