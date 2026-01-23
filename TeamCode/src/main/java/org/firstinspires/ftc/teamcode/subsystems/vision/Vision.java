package org.firstinspires.ftc.teamcode.subsystems.vision;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.command.SubsystemBase;
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
    private Drive drive;

    public Vision(final HardwareMap hardwareMap, Drive drive) {
        limelight = hardwareMap.get(Limelight3A.class, VisionConstants.limelightName);
        limelight.setPollRateHz(VisionConstants.pollRateHz);
        limelight.start();
        this.drive = drive;
    }

    public void calibrate() {
        List<LLResultTypes.FiducialResult> fiducialResults = limelight.getLatestResult().getFiducialResults();
        if (fiducialResults != null &&
                !fiducialResults.isEmpty() &&
                fiducialResults.get(0).getRobotPoseFieldSpace() != null &&
                (fiducialResults.get(0).getFiducialId() == 20 ||
                        fiducialResults.get(0).getFiducialId() == 24)) {
            Pose2D pose = Util.visionPoseToDWPose(fiducialResults.get(0).getRobotPoseFieldSpace());
            drive.setPose(pose);
            drive.setYawOffset(drive.getAlliance() == Drive.Alliance.BLUE ? Math.PI : 0);
        }
    }

    @Override
    public void periodic() {
        calibrate();
    }
}
