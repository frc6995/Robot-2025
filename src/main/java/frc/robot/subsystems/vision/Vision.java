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
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.util.TriConsumer;
public class Vision {
    public interface VisionConsumer extends TriConsumer<Pose2d, Double, Vector<N3>>{}
    private class Camera {
        String name;
        PhotonCamera camera;
        PhotonPoseEstimator estimator;
        PhotonCameraSim simCamera;
        Pose3d estimatedPose = new Pose3d();
        StructPublisher<Pose3d> rawPosePublisher;
        public Camera(
            String name,
            PhotonCamera camera,
            PhotonPoseEstimator estimator,
            PhotonCameraSim simCamera
        ) {
            this.name = name;
            this.camera = camera;
            this.estimator = estimator;
            this.simCamera = simCamera;
            var table = NetworkTableInstance.getDefault().getTable("Cameras").getSubTable(name);
            rawPosePublisher = table.getStructTopic("rawPose", Pose3d.struct).publish();
        }
        public void setRawPose(Pose3d pose) {
            estimatedPose = pose;
            rawPosePublisher.accept(estimatedPose);
        }
    };

    public class VisionConstants {
        private static SimCameraProperties cameraProp = new SimCameraProperties();
        static {
            // A 640 x 480 camera with a 100 degree diagonal FOV.
            cameraProp.setCalibration(1280, 900, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.12, 0.04);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);
        }
        public static final SimCameraProperties SIM_CAMERA_PROPERTIES = cameraProp;


        public static final Map<String, Transform3d> CAMERAS = Map.of(
            "OV9281-FL", new Transform3d(
                Units.inchesToMeters(4.875),
                Units.inchesToMeters(0),
                Units.inchesToMeters(7.25)-0.02,
                new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-34), Units.degreesToRadians(0))
            )
        );
        public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
    VisionSystemSim visionSim = new VisionSystemSim("main");
    private List<Camera> m_cameras;
    private List<PhotonCamera> m_actualCameras;
    private List<PhotonCameraSim> m_simCameras;
    private VisionConsumer addVisionMeasurement;
    private Supplier<Pose2d> getPose;
    public Vision(
        VisionConsumer addVisionMeasurement, Supplier<Pose2d> getPose) {
        if (RobotBase.isSimulation()) {
            visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
        }

        this.addVisionMeasurement = addVisionMeasurement;
        this.getPose = getPose;
        m_cameras = new ArrayList<>();
        VisionConstants.CAMERAS.entrySet().iterator().forEachRemaining((entry) -> {
            var translation = entry.getValue();
            var cam = new PhotonCamera(entry.getKey());
            var estimator = new PhotonPoseEstimator(
                            VisionConstants.FIELD_LAYOUT,
                            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                            translation);
            estimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT);
            PhotonCameraSim simCam = null;
            if (RobotBase.isSimulation()) {
                simCam = new PhotonCameraSim(cam, VisionConstants.SIM_CAMERA_PROPERTIES);
                visionSim.addCamera(simCam, translation);
            }
            m_cameras.add(new Camera(entry.getKey(), cam, estimator, simCam));
        });
    }

    public boolean filterPose(Pose3d robotPose) {
        if (Math.abs(robotPose.getZ()) > 0.5) {return false;}
        return true;
    }

    private void handleResult(Camera camera, PhotonPipelineResult result) {
        var estimator = camera.estimator;
        estimator.setReferencePose(getPose.get());
        
        var robotPoseOpt = estimator.update(result);
        if (robotPoseOpt.isEmpty()) {
            return;
        }
        camera.setRawPose(robotPoseOpt.get().estimatedPose);
        var pose = robotPoseOpt.get();
        this.addVisionMeasurement.accept(
            pose.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(pose.timestampSeconds),
            VecBuilder.fill(0.05, 0.05, 0.05));
        
    }

    public void update() {
        visionSim.update(getPose.get());
        for (Camera camera : m_cameras) {
            for (PhotonPipelineResult result : camera.camera.getAllUnreadResults()) {
                handleResult(camera, result);
            }
    }
}}
