package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.utils.PolarVector;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.function.BooleanSupplier;

public class TurretAlignCommand extends CommandBase {
    private Drive drive;
    private Turret turret;
    private Drive.Alliance alliance;
    private Boolean isAlign;
    private BooleanSupplier killButton;
    private Follower follower;

    public TurretAlignCommand(Drive drive, Turret turret, Drive.Alliance alliance,
                              BooleanSupplier killButton) {
        this.drive = drive;
        this.turret = turret;
        this.alliance = alliance;
        this.isAlign = true;
        this.killButton = killButton;
        addRequirements(turret);
    }

    public TurretAlignCommand(Follower follower, Turret turret, Drive.Alliance alliance) {
        this.follower = follower;
        this.turret = turret;
        this.alliance = alliance;
        this.isAlign = true;
        this.killButton = () -> false;
    }

    @Override
    public void initialize() {
        turret.setTurretState(Turret.TurretState.ACTIVE);
    }

    @Override
    public void execute() {
        if (killButton.getAsBoolean()) isAlign = !isAlign;
        if (isAlign) {
            if (drive != null) {
                PolarVector goalInRobotSys = Util.goalInRobotSys(drive.getPose(), alliance);
                turret.setTurret(goalInRobotSys.getHeading(DriveConstants.angleUnit));
            }
            else {
                PolarVector goalInRobotSys = Util.goalInRobotSys(Util.PoseToPose2D(follower.getPose()), alliance);
                turret.setTurret(goalInRobotSys.getHeading(DriveConstants.angleUnit));
            }
        }
        else {
            turret.setTurretState(Turret.TurretState.INIT);
        }
    }
}
