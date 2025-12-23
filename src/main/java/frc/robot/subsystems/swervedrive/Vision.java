package frc.robot.subsystems.swervedrive;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

/**
 * Vision class to handle QuestNav pose estimation via NetworkTables.
 */
public class Vision extends SubsystemBase {
  private final QuestNav questNav;
  private final Matrix<N3, N1> QUESTNAV_STD_DEVS;
  private final Transform3d QUESTNAV_TO_ROBOT;
  private final Pose3d resetPose;
  private final SwerveSubsystem swerveSubsystem;
  
  public Vision(Pose2d startingPose, Field2d field, SwerveSubsystem swerve)
  {
    swerveSubsystem = swerve;
    questNav = new QuestNav();
    Pose3d startingPose3d = new Pose3d(startingPose);
    QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );
    QUESTNAV_TO_ROBOT = new Transform3d();
    resetPose = startingPose3d.transformBy(QUESTNAV_TO_ROBOT);
    questNav.setPose(resetPose);
    field.getObject("QuestNavPose").setPose(startingPose);
    swerve.addVisionStandardDeviation(QUESTNAV_STD_DEVS);
  }

@Override
public void periodic() {
    // Get the latest pose data frames from the Quest
    PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

    // Loop over the pose data frames and send them to the pose estimator
    for (PoseFrame questFrame : questFrames) {
        // Make sure the Quest was tracking the pose for this frame
        if (questFrame.isTracking()) {
            // Get the pose of the Quest
            Pose3d questPose = questFrame.questPose3d();
            // Get timestamp for when the data was sent
            double timestamp = questFrame.dataTimestamp();

            

            // You can put some sort of filtering here if you would like!

            // Add the measurement to our estimator
            swerveSubsystem.addVisionReading(questPose.toPose2d(), timestamp);
        }
    }
}
}
