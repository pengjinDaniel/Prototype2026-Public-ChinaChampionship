package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Shooter;

public class ShootCommand extends CommandBase {
    Shooter shooter;
    Intake intake;

    public ShootCommand(Intake intake, Shooter shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        if (shooter.getPitchState() == Shooter.PitchState.HIGH) {
            intake.setIntakeState(Intake.IntakeState.FARSHOOT);
        }
        else {
            intake.setIntakeState(Intake.IntakeState.FORWARD);
        }
    }
}
