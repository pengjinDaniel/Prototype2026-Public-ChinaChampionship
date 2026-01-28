package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.utils.PolarVector;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.function.BooleanSupplier;

public class ShooterAlignCommand extends CommandBase {
    private Drive drive;
    private Shooter shooter;
    private Drive.Alliance alliance;
    private BooleanSupplier killButton;
    private boolean isAlign;

    public ShooterAlignCommand(Drive drive, Shooter shooter, Drive.Alliance alliance,
                               BooleanSupplier killButton) {
        this.drive = drive;
        this.shooter = shooter;
        this.alliance = alliance;
        this.killButton = killButton;
        isAlign = true;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(Shooter.ShooterState.DYNAMIC);
    }

    @Override
    public void execute() {
        if (killButton.getAsBoolean()) isAlign = !isAlign;
        if (isAlign && drive.getAligned()) {
            PolarVector goalInTurretSys = Util.goalInTurretSys(drive.getPose(), alliance);
            shooter.setDynamicSpeed(Util.getShooterVelocity(goalInTurretSys));
            if (goalInTurretSys.getMagnitude() > DriveConstants.nearGoalDistance + 30) {
                shooter.setPitchState(Shooter.PitchState.HIGH);
            }
            else if (goalInTurretSys.getMagnitude() < 30) {
                shooter.setPitchState(Shooter.PitchState.LOW);
            }
            else {
                shooter.setPitchState(Shooter.PitchState.MIDDLE);
            }
        }
        else {
            shooter.setShooterState(Shooter.ShooterState.SLOW);
        }
    }
}
