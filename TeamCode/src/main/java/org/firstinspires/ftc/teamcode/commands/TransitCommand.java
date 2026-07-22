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
    private static final double DEFAULT_SHOOT_TIME_MS = 750;
    private static final double FAR_SHOOT_TIME_MS = 1050;
    private final double configuredShootTimeMs;
    private double shootTimeMs;

    public TransitCommand(Shooter shooter, Transit transit, Intake intake) {
        this(shooter, transit, intake, Double.NaN);
    }

    public TransitCommand(Shooter shooter, Transit transit, Intake intake, double shootTimeMs) {
        this.shooter = shooter;
        this.transit = transit;
        this.intake = intake;
        this.configuredShootTimeMs = shootTimeMs;
        this.shootTimeMs = getInitialShootTimeMs();
        this.hasOpened = false;
        this.openTimer = new ElapsedTime();
    }

    private double getInitialShootTimeMs() {
        return Double.isNaN(configuredShootTimeMs) ? DEFAULT_SHOOT_TIME_MS : configuredShootTimeMs;
    }

    @Override
    public void initialize() {
        transit.setState(Transit.TransitState.CLOSE);
        hasOpened = false;
        shootTimeMs = getInitialShootTimeMs();
        openTimer.reset();
    }

    @Override
    public void execute() {
        if (shooter.isShooterAtSetPoint() && shooter.getShooterState() != Shooter.ShooterState.STOP) {
            if (!hasOpened) {
                transit.setState(Transit.TransitState.OPEN);
                hasOpened = true;
                openTimer.reset();
            }
            if (openTimer.milliseconds() > 150) {
                if (shooter.getPitchState() == Shooter.PitchState.FAR1
                        || shooter.getPitchState() == Shooter.PitchState.FAR2) {
                    intake.setIntakeState(Intake.IntakeState.FARSHOOT);
                    if (Double.isNaN(configuredShootTimeMs)) {
                        shootTimeMs = FAR_SHOOT_TIME_MS;
                    }
                }
                else intake.setIntakeState(Intake.IntakeState.FORWARD);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return hasOpened && openTimer.milliseconds() > shootTimeMs;
    }
}
