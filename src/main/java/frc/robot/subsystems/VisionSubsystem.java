package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem {
    public PhotonCamera photonCamera;
    public RobotPoseEstimator robotPoseEstimator;

    public VisionSubsystem() {
        AprilTagFieldLayout atfl;
        try {
            atfl = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");    
        }
        catch(Exception e) {
            System.out.println("Didn't read April Tag Field Layout");
            System.out.println(e);
        }

        photonCamera = new PhotonCamera(VisionConstants.cameraName);
        
    }
}
