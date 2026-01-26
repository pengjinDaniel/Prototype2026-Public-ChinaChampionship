package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

public class CalibrateCommand extends CommandBase {
    private Vision vision;
    private Drive drive;

    public CalibrateCommand(Vision vision, Drive drive) {
        this.vision = vision;
        this.drive = drive;

        addRequirements(vision);
    }

    @Override
    public void execute() {
        vision.calibrate(drive);
    }
}
