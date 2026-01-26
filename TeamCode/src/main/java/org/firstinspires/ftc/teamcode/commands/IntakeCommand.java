package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.transit.Transit;

public class IntakeCommand extends CommandBase {
    private Intake intake;
    private Transit transit;

    public IntakeCommand(Intake intake, Transit transit) {
        this.intake = intake;
        this.transit = transit;
    }

    @Override
    public void initialize() {
        transit.setState(Transit.TransitState.CLOSE);
    }

    @Override
    public void execute() {
        this.intake.setIntakeState(Intake.IntakeState.FORWARD);
    }

    @Override
    public void end(boolean interrupted) {
        this.intake.setIntakeState(Intake.IntakeState.STOP);
    }
}
