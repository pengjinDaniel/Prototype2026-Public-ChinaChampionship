package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;
import org.firstinspires.ftc.teamcode.utils.PolarVector;
import org.firstinspires.ftc.teamcode.utils.Util;

import java.util.function.BooleanSupplier;

public class ShooterAlignCommand extends CommandBase {
    private Drive drive;
    private Shooter shooter;
    private Drive.Alliance alliance;
    private BooleanSupplier killButton;
    private Follower follower;
    private boolean isAlign;
    private Transit transit;

    public ShooterAlignCommand(Drive drive, Shooter shooter, Transit transit, Drive.Alliance alliance,
                               BooleanSupplier killButton) {
        this.drive = drive;
        this.shooter = shooter;
        this.alliance = alliance;
        this.killButton = killButton;
        this.transit = transit;
        isAlign = true;
        addRequirements(shooter);
    }

    public ShooterAlignCommand(Follower follower, Shooter shooter, Transit transit, Drive.Alliance alliance) {
        this.follower = follower;
        this.shooter = shooter;
        this.alliance = alliance;
        this.killButton = () -> false;
        this.transit = transit;
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
        if (isAlign) {
            PolarVector goalInTurretSys = new PolarVector(0, 0);
            if (drive != null && drive.getAligned()) {
                goalInTurretSys = Util.goalInTurretSys(drive.getPose(), alliance);
            }
            if (follower != null) {
                goalInTurretSys = Util.goalInTurretSys(Util.PoseToPose2D(follower.getPose()), alliance);
            }

            if (transit.getState() != Transit.TransitState.OPEN)
                shooter.setDynamicSpeed(Util.getShooterVelocity(goalInTurretSys));
            if (goalInTurretSys.getMagnitude() > 100) {
                shooter.setPitchState(Shooter.PitchState.HIGH);
            }
            else if (goalInTurretSys.getMagnitude() < 46) {
                shooter.setPitchState(Shooter.PitchState.LOW);
            }
            else if (goalInTurretSys.getMagnitude() > 46.6) {
                shooter.setPitchState(Shooter.PitchState.MIDDLE);
            }
        }
    }
}
