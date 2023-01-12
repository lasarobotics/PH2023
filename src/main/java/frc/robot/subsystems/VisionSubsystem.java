package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.Optional;

import javax.xml.crypto.dsig.Transform;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem {
    public PhotonCamera photonCamera;
    public RobotPoseEstimator robotPoseEstimator;

    public VisionSubsystem() throws Exception {
        AprilTagFieldLayout atfl;
        try {
            atfl = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");    
            photonCamera = new PhotonCamera(VisionConstants.cameraName);
            ArrayList camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
            camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));
        }
        catch(Exception e) {
            System.out.println("Didn't read April Tag Field Layout");
            throw new Exception(e);
        }
    }

    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();

        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();

        if(result.isPresent()){
            return new Pair<Pose2d,Double>(result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
        }
        else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
