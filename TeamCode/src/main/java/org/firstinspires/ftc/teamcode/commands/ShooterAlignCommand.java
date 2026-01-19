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
    private Boolean isAlign;
    private BooleanSupplier killButton;

    public ShooterAlignCommand(Drive drive, Shooter shooter, Drive.Alliance alliance,
                               BooleanSupplier killButton) {
        this.drive = drive;
        this.shooter = shooter;
        this.alliance = alliance;
        this.isAlign = true;
        this.killButton = killButton;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.setShooterState(Shooter.ShooterState.DYNAMIC);
    }

    @Override
    public void execute() {
        if (killButton.getAsBoolean()) isAlign = !isAlign;
        if (isAlign) {
            PolarVector goalInRobotSys = Util.goalInRobotSys(drive.getExpectedPose(alliance), alliance);
            shooter.setDynamicSpeed(Util.getShooterVelocity(goalInRobotSys));
            if (goalInRobotSys.getMagnitude() > DriveConstants.nearGoalDistance + 30) {
                shooter.setPitchState(Shooter.PitchState.HIGH);
            }
            else if (goalInRobotSys.getMagnitude() < 30) {
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
