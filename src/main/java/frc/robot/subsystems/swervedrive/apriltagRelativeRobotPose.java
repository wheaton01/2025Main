package frc.robot.subsystems.swervedrive;

/** 
 * Class representing a 3D pose relative to an AprilTag.
 * This pose includes the X, Y, and Angle (theta) components.
 */
public class ApriltagRelativeRobotPose {

    // Variables to store the pose values
    private double tX;  // X position relative to the AprilTag
    private double tY;  // Y position relative to the AprilTag
    private double tA;  // Angle (theta) relative to the AprilTag

    // Constructor to initialize the pose
    public ApriltagRelativeRobotPose(double tX, double tY, double tA) {
        this.tX = tX;
        this.tY = tY;
        this.tA = tA;
    }

    // Getter methods for each pose component
    public double getTyaw() {
        return tX;
    }

    public double getTpitch() {
        return tY;
    }

    public double getTA() {
        return tA;
    }

    // Optional: Override toString for easier printing
    @Override
    public String toString() {
        return "ApriltagRelativeRobotPose{" +
               "tX=" + tX +
               ", tY=" + tY +
               ", tA=" + tA +
               '}';
    }
}
