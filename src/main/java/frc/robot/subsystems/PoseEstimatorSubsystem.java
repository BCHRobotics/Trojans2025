package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;;

public class PoseEstimatorSubsystem extends SubsystemBase{

    private final PhotonCamera photonCamera;
    private final Drivetrain drivetrainSubsystem;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    
    // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others. 
    // This in turn means the particualr component will have a stronger influence
    // on the final pose estimate.
  
    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
     */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    
    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));
  
    private final SwerveDrivePoseEstimator poseEstimator;
  
    private final Field2d field2d = new Field2d();
  
    private double previousPipelineTimestamp = 0;

    public static final Transform3d kRobotToCam = 
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), 
        new Rotation3d(0, -0, 0));
  
    public PoseEstimatorSubsystem(PhotonCamera photonCamera, Drivetrain drivetrainSubsystem) {
      this.photonCamera = photonCamera;
      this.drivetrainSubsystem = drivetrainSubsystem;
      AprilTagFieldLayout layout;
      layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

      var alliance = DriverStation.getAlliance().orElse(null);
      layout.setOrigin(alliance == Alliance.Blue ?
          OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);
      this.aprilTagFieldLayout = layout;
  
      poseEstimator =  new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          drivetrainSubsystem.getGyroscopeRotation(),
          drivetrainSubsystem.getModulePositions(),
          new Pose2d(),
          stateStdDevs,
          visionMeasurementStdDevs);

        
      SmartDashboard.putData("Field", field2d);
      SmartDashboard.putString("Pose",  this.getFormattedPose());
    }
  
    @Override
    public void periodic() {
      // Update pose estimator with the best visible target
      var pipelineResult = photonCamera.getLatestResult();
      var resultTimestamp = pipelineResult.getTimestampSeconds();
      if (resultTimestamp != previousPipelineTimestamp && pipelineResult.hasTargets()) {
        previousPipelineTimestamp = resultTimestamp;
        var target = pipelineResult.getBestTarget();
        var fiducialId = target.getFiducialId();
        // Get the tag pose from field layout - consider that the layout will be null if it failed to load
        Optional<Pose3d> tagPose = aprilTagFieldLayout == null ? Optional.empty() : aprilTagFieldLayout.getTagPose(fiducialId);
        if (target.getPoseAmbiguity() <= .2 && fiducialId >= 0 && tagPose.isPresent()) {
          var targetPose = tagPose.get();
          Transform3d camToTarget = target.getBestCameraToTarget();
          Pose3d camPose = targetPose.transformBy(camToTarget.inverse());
  
          var visionMeasurement = camPose.transformBy(kRobotToCam);
          poseEstimator.addVisionMeasurement(visionMeasurement.toPose2d(), resultTimestamp);
        }
      }
      // Update pose estimator with drivetrain sensors
      poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions());
  
      field2d.setRobotPose(getCurrentPose());
    }
  
    private String getFormattedPose() {
      var pose = getCurrentPose();
      return String.format("(%.2f, %.2f) %.2f degrees", 
          pose.getX(), 
          pose.getY(),
          pose.getRotation().getDegrees());
    }
  
    public Pose2d getCurrentPose() {
      return poseEstimator.getEstimatedPosition();
    }
  
    public Supplier<Pose2d> getCurrentPoseSupplier() {

        Supplier<Pose2d> poseSupplier = this::getCurrentPose;

        return poseSupplier;
      }
    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's position on the field is known, like at the beginning of
     * a match.
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
      poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
    }
  
    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
      setCurrentPose(new Pose2d());
    }
}
