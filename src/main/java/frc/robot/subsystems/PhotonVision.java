package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.FieldConstants;
import frc.robot.Constants.CAMERA;

public class PhotonVision {
   // Creates a new PhotonCamera.
   public PhotonCamera m_limelight = new PhotonCamera("limelight");
   public PhotonCamera m_HD3000 = new PhotonCamera("lifecam");
   public SimVisionSystem ballvisionSys;
   public SimVisionSystem shootervisionSys;

   public PhotonVision() {
      m_limelight.setPipelineIndex(CAMERA.LIMELIGHTPIPELINE);
      m_HD3000.setPipelineIndex(CAMERA.HD3000PIPELINE);

      double ballcamDiagFOV = 75.0; // degrees
      double shootercamDiagFOV = 75.0; // degrees
      Transform3d ballcameraToRobot = new Transform3d(new Translation3d(0.0, 0.0, CAMERA.BALLCAMERAHEIGHT), new Rotation3d(0,CAMERA.BALLCAMERAANGLE,0)); // meters
      Transform3d shootercameraToRobot = new Transform3d(new Translation3d(0.0, 0.0, CAMERA.SHOOTERCAMERAHEIGHT), new Rotation3d(0,CAMERA.SHOOTERCAMERAANGLE,0)); // meters
      double maxLEDRange = 20;          // meters
      int ballcamResolutionWidth = 640;     // pixels
      int ballcamResolutionHeight = 480;    // pixels
      double ballminTargetArea = 10;        // square pixels
      int shootercamResolutionWidth = 640;     // pixels
      int shootercamResolutionHeight = 480;    // pixels
      double shooterminTargetArea = 10;        // square pixels
      

      ballvisionSys = new SimVisionSystem("lifecam",
                                    ballcamDiagFOV,
                                    ballcameraToRobot,
                                    9000, // does not use LEDs
                                    ballcamResolutionWidth,
                                    ballcamResolutionHeight,
                                    ballminTargetArea);

      shootervisionSys = new SimVisionSystem("limelight",
                                    shootercamDiagFOV,
                                    shootercameraToRobot,
                                    maxLEDRange,
                                    shootercamResolutionWidth,
                                    shootercamResolutionHeight,
                                    shooterminTargetArea);

      double tgtXPos = Units.feetToMeters(54 / 2);
      double tgtYPos = Units.feetToMeters(27 / 2);
      var targetPose = new Pose3d(new Translation3d(tgtXPos, tgtYPos, FieldConstants.visionTargetHeightLower), new Rotation3d(0, -21.0, 0)); // meters
      double balltargetWidth = Units.inchesToMeters(9.5);
      double shootertargetWidth = Units.inchesToMeters(36); // Actually 4ft wide but this is a straight replacement for a curved goal
      double shootertargetHeight = Units.inchesToMeters(2);
      
      var ball = new Pose3d(FieldConstants.cargoD.getX(), FieldConstants.cargoD.getY(), 0, new Rotation3d());
      var ballTgt = RectangularSimVisionTarget(ball,
                                       0,
                                       balltargetWidth,
                                       balltargetWidth, // same as height
                                       balltargetWidth);

      var shooterTgt = RectangularSimVisionTarget(targetPose,
                                       FieldConstants.visionTargetHeightLower,
                                       shootertargetWidth,
                                       shootertargetHeight,
                                       shootertargetWidth); // width is same as depth since 4ft circle
      
      for (int i = 0; i < ballTgt.size(); i++) {
         ballvisionSys.addSimVisionTarget(ballTgt.get(i));
      }
      for (int i = 0; i < shooterTgt.size(); i++) {
         shootervisionSys.addSimVisionTarget(shooterTgt.get(i));
      }

      NetworkTableInstance.getDefault().getTable("photonvision").getEntry("version").setValue("v2022.1.4");
   }

   public void fieldSetup(Field2d field) {
      var ball = field.getObject("ball");
      var hub = field.getObject("hub");

      ball.setPose(FieldConstants.cargoD.getX(), FieldConstants.cargoD.getY(), Rotation2d.fromDegrees(0));
      hub.setPose(FieldConstants.hubCenter.getX(), FieldConstants.hubCenter.getY(), Rotation2d.fromDegrees(-21));
   }

   public void lightsOn() {
      m_limelight.setLED(VisionLEDMode.kOn);
   }

   public void lightsOff() {
      m_limelight.setLED(VisionLEDMode.kOff);
   }

   public double getYaw() {
      var result = m_limelight.getLatestResult();
      if (result.hasTargets()) {
         return result.getBestTarget().getYaw();
      }
      return -999.0;
   }

   // Both of these are dangerous and need "hasTargets" needs to be checked before using
   public double distanceToBallTarget(PhotonPipelineResult result) {
      return PhotonUtils.calculateDistanceToTargetMeters(CAMERA.BALLCAMERAHEIGHT,
                  CAMERA.BALLTARGETHEIGHT,
                  CAMERA.BALLCAMERAANGLE,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));
   }

   public double distanceToShooterTarget(PhotonPipelineResult result) {
      return PhotonUtils.calculateDistanceToTargetMeters(CAMERA.SHOOTERCAMERAHEIGHT,
                  FieldConstants.visionTargetHeightLower,
                  CAMERA.SHOOTERCAMERAANGLE,
                  Units.degreesToRadians(result.getBestTarget().getPitch()));
   }

   public List<SimVisionTarget> RectangularSimVisionTarget(
         Pose3d targetPos,
         double targetHeightAboveGroundMeters,
         double targetWidthMeters,
         double targetHeightMeters,
         double targetDepthMeters) {
      List<SimVisionTarget> targetList = new ArrayList<SimVisionTarget>();
      
      var targetPos1 = targetPos.transformBy(new Transform3d(
         new Translation3d(0, -targetDepthMeters/2, targetHeightAboveGroundMeters).rotateBy(targetPos.getRotation()),
         new Rotation3d(0, 180.0, 0)));
   
      var targetPos2 = targetPos.transformBy(new Transform3d(
         new Translation3d(-targetWidthMeters/2, 0, targetHeightAboveGroundMeters).rotateBy(targetPos.getRotation()),
         new Rotation3d(0, -90.0, 0)));

      var targetPos3 = targetPos.transformBy(new Transform3d(
         new Translation3d(0, targetDepthMeters/2, targetHeightAboveGroundMeters).rotateBy(targetPos.getRotation()),
         new Rotation3d()));

      var targetPos4 = targetPos.transformBy(new Transform3d(
         new Translation3d(targetWidthMeters/2, 0, targetHeightAboveGroundMeters).rotateBy(targetPos.getRotation()),
         new Rotation3d(0, 90.0, 0)));

      targetList.add(new SimVisionTarget(targetPos1, // Intial face
            targetWidthMeters,
            targetHeightMeters,
            10));
      targetList.add(new SimVisionTarget(targetPos2, // Left face
            targetDepthMeters, // On the side width is the inital depth
            targetHeightMeters,
            11));
      targetList.add(new SimVisionTarget(targetPos3, // Back face
            targetWidthMeters,
            targetHeightMeters,
            12));
      targetList.add(new SimVisionTarget(targetPos4, // Right face
            targetDepthMeters, // On the side width is the inital depth
            targetHeightMeters,
            13));
      return targetList;
   }
}
