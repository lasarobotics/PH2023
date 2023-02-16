// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.RobotPoseEstimator;

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
    private PhotonCamera forwardCamera, reverseCamera;

    public Hardware(PhotonCamera forwardCamera, PhotonCamera reverseCamera) {
      this.forwardCamera = forwardCamera;
      this.reverseCamera = reverseCamera;
    }
  }

  static class ForwardCamera {
    static final Transform3d LOCATION =
      new Transform3d(
        new Translation3d(0.0, 0.0, 1.0),
        new Rotation3d(0, 0,0)
      );
    static final String NAME = "forwardCamera";
  }

  static class ReverseCamera {
    static final Transform3d LOCATION =
      new Transform3d(
        new Translation3d(0.0, 0.0, 1.0),
        new Rotation3d(0, 0,180)
      );
    static final String NAME = "reverseCamera";
  }
  
  private static VisionSubsystem m_subsystem;

  private PhotonCamera m_forwardCamera;
  private PhotonCamera m_reverseCamera;
  private AprilTagFieldLayout m_fieldLayout;
  private RobotPoseEstimator m_poseEstimator;

  /**
   * Create a new vision subsystem
   * @param visionHardware Vision hardware
   */
  private VisionSubsystem(Hardware visionHardware) {
    this.m_forwardCamera = visionHardware.forwardCamera;
    this.m_reverseCamera = visionHardware.reverseCamera;

    try {
      m_fieldLayout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    } catch (IOException e) { e.printStackTrace(); }

    var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
    camList.add(new Pair<PhotonCamera, Transform3d>(m_forwardCamera, ForwardCamera.LOCATION));
    camList.add(new Pair<PhotonCamera, Transform3d>(m_reverseCamera, ReverseCamera.LOCATION));

    m_poseEstimator = new RobotPoseEstimator(m_fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
  }

  public static VisionSubsystem getInstance() {
    if (m_subsystem == null) m_subsystem = new VisionSubsystem(VisionSubsystem.initializeHardware());
    return m_subsystem;
  }

  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return this.m_fieldLayout;
  }

  public static Hardware initializeHardware() {
    Hardware visionHardware = new Hardware(new PhotonCamera(ForwardCamera.NAME), new PhotonCamera(ReverseCamera.NAME));

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
