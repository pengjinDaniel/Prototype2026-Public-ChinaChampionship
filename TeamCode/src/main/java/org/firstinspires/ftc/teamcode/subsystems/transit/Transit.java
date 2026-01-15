package org.firstinspires.ftc.teamcode.subsystems.transit;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Transit extends SubsystemBase {
    public final Servo limitServo;
    public LimitServoState limitState = LimitServoState.CLOSE;

    public Transit(HardwareMap hardwareMap) {
        limitServo = hardwareMap.get(Servo.class, TransitConstants.limitServoName);
    }

    public enum LimitServoState {
        CLOSE(TransitConstants.limitServoClosePos),
        OPEN(TransitConstants.limitServoOpenPos);

        final double servoPos;

        LimitServoState(double limitServoPos) {
            servoPos = limitServoPos;
        }
    }


    public void setLimitServoState(LimitServoState limitServoState) {
        limitState = limitServoState;
    }

    @Override
    public void periodic() {
        limitServo.setPosition(limitState.servoPos);
    }
}
