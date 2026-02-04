package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final Servo limitServo;
    public TransitState transitState = TransitState.CLOSE;

    public Transit(HardwareMap hardwareMap) {
        limitServo = hardwareMap.get(Servo.class, TransitConstants.transitServoName);
    }

    public static enum TransitState {
        CLOSE(TransitConstants.transitServoClosePos),
        OPEN(TransitConstants.transitServoOpenPos);

        final double servoPos;

        TransitState(double limitServoPos) {
            servoPos = limitServoPos;
        }
    }


    public void setState(TransitState transitState) {
        this.transitState = transitState;
    }

    public TransitState getState() {
        return this.transitState;
    }

    @Override
    public void periodic() {
        limitServo.setPosition(transitState.servoPos);
    }
}
