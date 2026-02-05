package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class TransitCommand extends CommandBase {
    private final Shooter shooter;
    private final Transit transit;
    private final Intake intake;
    private boolean hasOpened;
    private final ElapsedTime openTimer;
    private static double delay = 600;

    public TransitCommand(Shooter shooter, Transit transit, Intake intake) {
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
        this.hasOpened = false;
        this.openTimer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        transit.setState(Transit.TransitState.CLOSE);
        hasOpened = false;
        openTimer.reset();
    }

    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint() && shooter.getShooterState() != Shooter.ShooterState.STOP) {
            if (shooter.getPitchState() == Shooter.PitchState.FAR1
                    || shooter.getPitchState() == Shooter.PitchState.FAR2) {
                intake.setIntakeState(Intake.IntakeState.FARSHOOT);
                delay = 900;
            }
            else intake.setIntakeState(Intake.IntakeState.FORWARD);

            if (!hasOpened) {
                transit.setState(Transit.TransitState.OPEN);
                hasOpened = true;
                openTimer.reset();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return hasOpened && openTimer.milliseconds() > delay;
    }
}