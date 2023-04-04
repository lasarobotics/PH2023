// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;

public class VisionSubsystem {
  public static class Hardware {
    private PhotonCamera forwardCamera;

    public Hardware(PhotonCamera forwardCamera) {
      this.forwardCamera = forwardCamera;
    }
  }

  static class ForwardCamera {
    static final Transform3d LOCATION =
      new Transform3d(
        new Translation3d(0.0, 0.2032, 0.9144),
        new Rotation3d(0, 0,0)
      );
    static final String NAME = "forwardCamera";
  }
  
  private static VisionSubsystem m_subsystem;

  private PhotonCamera m_forwardCamera;
  private AprilTagFieldLayout m_fieldLayout;
  private RobotPoseEstimator m_poseEstimator;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   */
  private VisionSubsystem(Hardware visionHardware) {
    this.m_forwardCamera = visionHardware.forwardCamera;

    try {
      m_fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) { e.printStackTrace(); }

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(m_forwardCamera, ForwardCamera.LOCATION));

    m_poseEstimator = new RobotPoseEstimator(m_fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);

    // Set vision pipeline
   m_forwardCamera.setPipelineIndex(0);
  }

  public static VisionSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new VisionSubsystem(VisionSubsystem.initializeHardware());
    return m_subsystem;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return this.m_fieldLayout;
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(new PhotonCamera(ForwardCamera.NAME));

    return visionHardware;
  }

  /**
   * 
   * @param prevEstimatedRobotPose The current best guess at robot pose
   * @return  A pair of the fused camera observations to a single Pose2d on the field, and the time
   *          of the observation. Assumes a planar field and the robot is always firmly on the ground
   */
  public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_poseEstimator.setReferencePose(prevEstimatedRobotPose);

    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = m_poseEstimator.update();
    if (result.isPresent()) {
      if (result.get().getFirst() != null)
        return new Pair<Pose2d, Double>(
                result.get().getFirst().toPose2d(), currentTime - result.get().getSecond());
      else return new Pair<Pose2d, Double>(null, 0.0); 
    } else {
          return new Pair<Pose2d, Double>(null, 0.0);
      }
  }
}
