package org.firstinspires.ftc.teamcode.subsystems.led;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Led extends SubsystemBase {
    public Servo led;

    public enum LedState {
        YELLOW(0.4),
        GREEN(0.5);

        public double pos;

        LedState(double pos) {
            this.pos = pos;
        }
    }

    public LedState ledState;

    public Led(HardwareMap hardwareMap) {
        this.led = hardwareMap.get(Servo.class, "led");
        ledState = LedState.YELLOW;
    }

    public void setLedState(LedState ledState) {
        this.ledState = ledState;
    }

    @Override
    public void periodic() {
        led.setPosition(ledState.pos);
    }
}
