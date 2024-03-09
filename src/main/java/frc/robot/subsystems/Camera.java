// package frc.robot.subsystems;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;
// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// public class Camera extends SubsystemBase {
// private PhotonCamera camera;
// private Transform3d toRobot;
// private AprilTagFieldLayout field;
// private List<Transform3d> distances;
// private PhotonPipelineResult result;

// // constructor, toRobot is the position of the camera if robot's center is
// // origin
// public Camera(String name, Transform3d toRobot, AprilTagFieldLayout field) {
// camera = new PhotonCamera(name);
// distances = new ArrayList<Transform3d>();
// this.field = field;
// this.toRobot = toRobot;
// }

// // update data
// public void update() {
// result = camera.getLatestResult();

// if (!distances.isEmpty()) {
// distances.clear();
// }

// if (result.hasTargets()) {
// for (PhotonTrackedTarget target : result.getTargets()) {
// if (target.getPoseAmbiguity() < 0.2) {
// distances.add(target.getBestCameraToTarget());
// System.out.println(target.getBestCameraToTarget());
// distances.add(target.getAlternateCameraToTarget());
// }
// }
// }
// }

// // get list of distances
// public List<Transform3d> getDistances() {
// return distances;
// }

// // get robot poses
// public List<Pose3d> getPoses() {
// List<Pose3d> poses = new ArrayList<Pose3d>();

// if (result.hasTargets()) {
// for (PhotonTrackedTarget target : result.getTargets()) {
// poses.add(field.getTagPose(target.getFiducialId()).get().plus(target.getBestCameraToTarget().inverse())
// .plus(toRobot));
// poses.add(field.getTagPose(target.getFiducialId()).get()
// .plus(target.getAlternateCameraToTarget().inverse()).plus(toRobot));
// }
// }

// return poses;
// }

// // get list of detected targets
// public List<PhotonTrackedTarget> getTargets() {
// if (result.hasTargets()) {
// return result.getTargets();
// } else
// return null;
// }

// // get list of ids detected
// public List<Integer> getIDs() {
// if (result.hasTargets()) {
// List<Integer> ids = new ArrayList<Integer>();
// for (PhotonTrackedTarget target : result.getTargets()) {
// ids.add(target.getFiducialId());
// }
// return ids;
// } else
// return null;
// }

// // what side are we on
// public int getSpeakerTag() {
// Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
// int speakerTag = 0;

// if (alliance.isPresent()) {
// if (alliance.get() == DriverStation.Alliance.Red) {
// speakerTag = 4;
// } else {
// speakerTag = 7;
// }
// }

// return speakerTag;
// }
// }
