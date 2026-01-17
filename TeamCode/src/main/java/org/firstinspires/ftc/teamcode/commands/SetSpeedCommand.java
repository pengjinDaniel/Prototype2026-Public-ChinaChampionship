package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.PolarVector;
import org.firstinspires.ftc.teamcode.utils.Util;

public class SetSpeedCommand extends CommandBase {
    private Drive drive;
    private Turret turret;
    private Shooter shooter;
    private Drive.Alliance alliance;

    public SetSpeedCommand(Drive drive, Turret turret, Shooter shooter, Drive.Alliance alliance) {
        this.drive = drive;
        this.turret = turret;
        this.shooter = shooter;
        this.alliance = alliance;
    }

    @Override
    public void execute() {
        PolarVector goalInRobotSys = Util.goalInRobotSys(drive.getExpectedPose(alliance), alliance);
        turret.setTurret(goalInRobotSys.getHeading(DriveConstants.angleUnit));
        shooter.setDynamicSpeed(Util.getShooterVelocity(goalInRobotSys));
    }
}
