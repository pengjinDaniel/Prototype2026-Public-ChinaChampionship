package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.angleUnit;
import static org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants.distanceUnit;

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
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.List;

public class Vision extends SubsystemBase {
    private final Limelight3A limelight;

    public Vision(final HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.start();
    }

    public LLResultTypes.FiducialResult getFiducialResult() {
        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();
        if (fiducialResults == null) return null;
        if (fiducialResults.isEmpty()) return null;
        return fiducialResults.get(0);
    }

    public Pose2D getVisionPose() {
        LLResultTypes.FiducialResult fiducialResult = getFiducialResult();
        if (fiducialResult == null) return new Pose2D(distanceUnit, -114514, -114514, angleUnit, 0);
        return Util.visionPoseToDWPose(fiducialResult.getRobotPoseFieldSpace());
    }

    public boolean calibrate(Drive drive, Turret turret) {
        Pose2D pose = getVisionPose();
        if (pose.getX(distanceUnit) < -1e5) return false;
        drive.setPose(Util.turretToDrivePose(pose, turret));
        drive.setYawOffset(drive.getAlliance() == Drive.Alliance.BLUE ? Math.PI : 0);
        drive.setAligned(true);
        return true;
    }

    public boolean autoCalibrate(Follower follower, Turret turret) {
        Pose2D temp = getVisionPose();
        if (temp.getX(distanceUnit) < -1e5) return false;
        Pose pose = Util.Pose2DToPose(Util.turretToDrivePose(temp, turret));
        follower.setPose(pose);
        return true;
    }
}
