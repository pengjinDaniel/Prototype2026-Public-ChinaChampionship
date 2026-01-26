package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private Shooter shooter;
    private Transit transit;
    private boolean readyToShoot;

    public TransitCommand(Shooter shooter, Transit transit) {
        this.shooter = shooter;
        this.transit = transit;
        this.readyToShoot = false;
    }

    @Override
    public void initialize() {
        transit.setState(Transit.TransitState.CLOSE);
    }

    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint() && shooter.getShooterState() != Shooter.ShooterState.STOP) {
            transit.setState(Transit.TransitState.OPEN);
            readyToShoot = true;
        }
    }

    @Override
    public boolean isFinished() {
        return readyToShoot;
    }
}
