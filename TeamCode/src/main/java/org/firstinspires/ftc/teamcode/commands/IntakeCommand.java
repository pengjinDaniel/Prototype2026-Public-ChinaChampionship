package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void execute() {
        this.intake.setRunning(true);
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setRunning(false);
    }
}
