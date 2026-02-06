package org.firstinspires.ftc.teamcode.commands;

import android.view.Window;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.led.Led;

public class LedWinkCommand extends CommandBase {
    public Led led;
    public ElapsedTime timer;

    public LedWinkCommand(Led led) {
        this.led = led;
        this.timer = new ElapsedTime();
    }

    @Override
    public void initialize() {
        this.timer.reset();
    }

    @Override
    public void execute() {
        led.setLedState(Led.LedState.GREEN);
    }

    @Override
    public void end(boolean interrupted) {
        led.setLedState(Led.LedState.YELLOW);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= 200;
    }
}
