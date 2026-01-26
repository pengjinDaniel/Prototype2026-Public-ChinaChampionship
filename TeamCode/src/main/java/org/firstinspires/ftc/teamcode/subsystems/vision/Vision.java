package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.List;

public class Vision extends SubsystemBase {
    private Limelight3A limelight;

    public Vision(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.start();
    }

    public LLResultTypes.FiducialResult getFiducialResult() {
        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();
        if (fiducialResults == null) return null;
        if (fiducialResults.isEmpty()) return null;
        if (fiducialResults.get(0).getFiducialId() != 20 &&
                fiducialResults.get(0).getFiducialId() != 24) return null;
        return limelight.getLatestResult().getFiducialResults().get(0);
    }

    public void calibrate(Drive drive) {
        LLResultTypes.FiducialResult fiducialResult = getFiducialResult();
        if (fiducialResult == null) return;
        Pose2D pose = Util.visionPoseToDWPose(fiducialResult.getRobotPoseFieldSpace());
        drive.setPose(pose);
        drive.setYawOffset(drive.getAlliance() == Drive.Alliance.BLUE ? Math.PI : 0);
    }

    public void autoCalibrate(Follower follower) {
        LLResultTypes.FiducialResult fiducialResult = getFiducialResult();
        if (fiducialResult == null) return;
        Pose pose = Util.Pose2DToPose(Util.visionPoseToDWPose(fiducialResult.getRobotPoseFieldSpace()));
        follower.setPose(pose);
    }
}
