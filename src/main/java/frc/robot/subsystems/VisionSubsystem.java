// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  PhotonCamera camera;
  PhotonPipelineResult cameraFeed;
  List<PhotonTrackedTarget> targets;
  List<TargetCorner> corners;
  double error = 0;
  double x1;
  double x2;
  double xAvg;
  double counter;
  double corner0;
  double corner1;
  double corner2;
  double corner3;
  double[] cornerArrays;
  public VisionSubsystem() {
    camera = new PhotonCamera("C270_HD_WEBCAM");
  }

  public double getError() {
    cameraFeed = camera.getLatestResult();
    targets = cameraFeed.getTargets();
    for(PhotonTrackedTarget target : targets) {
      if(target.getFiducialId()==Constants.speakerTagID) {
        corners = target.getDetectedCorners();
        x1 = (corners.get(0).x + corners.get(3).x)/2;
        x2 = (corners.get(1).x + corners.get(2).x)/2;
        xAvg = (x1+x2)/2;
        error = xAvg;
      }
      else {
        error = 0;
      }
    }
    return error;
  }
  
/* 
  public double getDistanceToTarget() {
    cameraFeed = camera.getLatestResult();
    range = PhotonUtils.calculateDistanceToTargetMeters(corner3, 1.222, corner1, 0);
  }
  */
/* 
  public double[] getCornerX() {
    cameraFeed = camera.getLatestResult();
    for(PhotonTrackedTarget target : targets) {
      corners = target.getDetectedCorners();
    }
    counter = 0;
    for (TargetCorner corner : corners) {
      if(counter == 0) {
        corner0 = corner.x;
      }
      else if (counter == 1) {
        corner1 = corner.x;
      }
      else if (counter == 2) {
        corner2 = corner.x;
      }
      else if (counter == 3) {
        corner3 = corner.x;
      }
      cornerArrays = new double[]{corner0, corner1, corner2, corner3};
    }
    return cornerArrays;
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("Target Error", error);
    //SmartDashboard.putNumberArray("cornerArray", cornerArrays);
    
  }
}
