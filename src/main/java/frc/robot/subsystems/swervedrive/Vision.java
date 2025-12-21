package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.Optional;
import java.util.function.Supplier;
import swervelib.SwerveDrive;

/**
 * Vision class to handle QuestNav pose estimation via NetworkTables.
 */
public class Vision {

  /**
   * QuestNav NetworkTable.
   */
  private final NetworkTable questNavTable;
  /**
   * Subscriber for the robot pose array.
   * Expected format: [x, y, z, qw, qx, qy, qz] (7 elements) or similar.
   * Adjust key as needed based on specific QuestNav implementation.
   */
  private final DoubleArraySubscriber poseSubscriber;

  /**
   * Current pose supplier.
   */
  private final Supplier<Pose2d> currentPose;
  /**
   * Field2d for debugging.
   */
  private final Field2d field2d;

  /**
   * Standard deviation for vision measurements.
   * Trust QuestNav highly if tracking is good.
   */
  private static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);

  /**
   * Constructor for the Vision class.
   *
   * @param currentPose Current pose supplier.
   * @param field       Current field.
   */
  public Vision(Supplier<Pose2d> currentPose, Field2d field) {
    this.currentPose = currentPose;
    this.field2d = field;

    questNavTable = NetworkTableInstance.getDefault().getTable("QuestNav");
    // Assuming the key is "Pose" or "RobotPose". Checking "Pose" first.
    // Format: [x, y, z, qw, qx, qy, qz] -> Position + Quaternion
    poseSubscriber = questNavTable.getDoubleArrayTopic("Pose").subscribe(new double[] {});
  }

  /**
   * Update the pose estimation inside of {@link SwerveDrive}.
   *
   * @param swerveDrive {@link SwerveDrive} instance.
   */
  public void updatePoseEstimation(SwerveDrive swerveDrive) {
    // Get latest observation
    TimestampedDoubleArray[] updates = poseSubscriber.readQueue();

    for (TimestampedDoubleArray update : updates) {
      double[] data = update.value;
      long timestamp = update.timestamp; // Microseconds
      double timestampSeconds = timestamp / 1_000_000.0;

      Optional<Pose3d> visionPose = decodePose(data);

      if (visionPose.isPresent()) {
        Pose2d pose2d = visionPose.get().toPose2d();

        // Update Field2d for visualization
        field2d.getObject("QuestNavPose").setPose(pose2d);

        // Add measurement to SwerveDrive
        // Use default standard deviations for now.
        swerveDrive.addVisionMeasurement(pose2d, timestampSeconds, kSingleTagStdDevs);
      }
    }
  }

  /**
   * Decodes the double array from NetworkTables into a Pose3d.
   * Handles 7-element (Pos + Quat) and 6-element (Pos + Euler) arrays.
   * 
   * @param data Array of doubles.
   * @return Optional Pose3d.
   */
  private Optional<Pose3d> decodePose(double[] data) {
    if (data.length == 7) {
      // Translation + Quaternion: [x, y, z, qw, qx, qy, qz]
      return Optional.of(new Pose3d(
          new Translation3d(data[0], data[1], data[2]),
          new Rotation3d(new Quaternion(data[3], data[4], data[5], data[6]))));
    } else if (data.length == 6) {
      // Translation + Euler (Roll-Pitch-Yaw or similar): [x, y, z, roll, pitch, yaw]
      // Assuming typical order; might need tuning.
      return Optional.of(new Pose3d(
          new Translation3d(data[0], data[1], data[2]),
          new Rotation3d(data[3], data[4], data[5])));
    }
    return Optional.empty();
  }
}
