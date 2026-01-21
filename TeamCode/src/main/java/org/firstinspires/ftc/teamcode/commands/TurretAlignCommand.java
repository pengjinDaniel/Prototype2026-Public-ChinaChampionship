package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

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

    public TurretAlignCommand(Drive drive, Turret turret, Drive.Alliance alliance,
                              BooleanSupplier killButton) {
        this.drive = drive;
        this.turret = turret;
        this.alliance = alliance;
        this.isAlign = true;
        this.killButton = killButton;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.setTurretState(Turret.TurretState.ACTIVE);
    }

    @Override
    public void execute() {
        if (killButton.getAsBoolean()) isAlign = !isAlign;
        if (isAlign) {
            PolarVector goalInRobotSys = Util.goalInRobotSys(drive.getPose(), alliance);
            turret.setTurret(goalInRobotSys.getHeading(DriveConstants.angleUnit));
//            turret.setTurret(0);
        }
        else {
            turret.setTurretState(Turret.TurretState.INIT);
        }
    }
}
