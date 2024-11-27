package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

public class Vision {
    public record VisionMeasurement (Pose2d pose, double timestamp, Vector<N3> stddevs){};
    public class VisionConstants {
        private static SimCameraProperties cameraProp = new SimCameraProperties();
        static {
            // A 640 x 480 camera with a 100 degree diagonal FOV.
            cameraProp.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
        }
        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = cameraProp;


        public static final Map<String, Transform3d> CAMERAS = Map.of(
            "Arducam_OV2311_USB_Camera", new Transform3d(
                Units.inchesToMeters(-4.75),
                Units.inchesToMeters(0),
                Units.inchesToMeters(25.25),
                new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-35), Units.degreesToRadians(180))
            )
        );
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    }
    VisionSystemSim visionSim = new VisionSystemSim("main");
    private List<Pair<String, PhotonPoseEstimator>> m_cameras;
    private List<PhotonCamera> m_actualCameras;
    private List<PhotonCameraSim> m_simCameras;
    private Consumer<VisionMeasurement> addVisionMeasurement;
    private Supplier<Pose2d> getPose;
    public Vision(
            Consumer<VisionMeasurement> addVisionMeasurement, Supplier<Pose2d> getPose) {
        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo));
        }

        this.addVisionMeasurement = addVisionMeasurement;
        this.getPose = getPose;
        m_cameras = new ArrayList<>();
        m_actualCameras = new ArrayList<>();
        m_simCameras = new ArrayList<>();
        VisionConstants.CAMERAS.entrySet().iterator().forEachRemaining((entry) -> {
            var translation = entry.getValue();
            var cam = new PhotonCamera(entry.getKey());
            var estimator = new PhotonPoseEstimator(
                            VisionConstants.FIELD_LAYOUT,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            translation);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
            if (RobotBase.isSimulation()) {
                var simCam = new PhotonCameraSim(cam, VisionConstants.SIM_CAMERA_PROPERTIES);
                m_simCameras.add(simCam);
                visionSim.addCamera(simCam, translation);
            }
            m_actualCameras.add(cam);
            m_cameras.add(new Pair<String, PhotonPoseEstimator>(entry.getKey(), estimator));
        });
    }

    public boolean filterPose(Pose3d robotPose) {
        if (Math.abs(robotPose.getZ()) > 0.5) {return false;}
        return true;
    }

    public void update() {
        visionSim.update(getPose.get());
    }
}
