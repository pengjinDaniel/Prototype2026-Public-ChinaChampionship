package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.utils.Util;

public class VisionCalibrateCommand extends CommandBase {
    Drive drive;
    Vision vision;
    boolean finished;

    public VisionCalibrateCommand(Drive drive, Vision vision) {
        this.drive = drive;
        this.vision = vision;
        finished = false;
    }

    @Override
    public void execute() {
        Pose3D visionPose = vision.getRobotPosition();
        if (visionPose != null){
            drive.setPose(Util.visionPoseToDWPose(visionPose));
        }
        drive.setYawOffset(drive.getAlliance() == Drive.Alliance.BLUE ? Math.PI : 0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
