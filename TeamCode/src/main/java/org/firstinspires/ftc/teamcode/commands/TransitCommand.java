package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private Shooter shooter;
    private Transit transit;

    private Intake intake;

    public TransitCommand(Shooter shooter, Transit transit, Intake intake) {
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        transit.setState(Transit.TransitState.CLOSE);
        intake.setRunning(false);
    }

    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint() && shooter.getShooterState() != Shooter.ShooterState.STOP) {
            intake.setRunning(true);
            transit.setState(Transit.TransitState.OPEN);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setRunning(false);
    }
}
