package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake extends SubsystemBase {
    private final DcMotor intakeMotor;
    public IntakeState intakeState;

    public enum IntakeState {
        STOP(0),
        FORWARD(IntakeConstants.intakePower),
        FARSHOOT(0.9),
        REVERSED(-IntakeConstants.intakePower);

        public double power;

        IntakeState(double power) {
            this.power = power;
        }
    }

    public Intake(HardwareMap hardwareMap) {
        this.intakeMotor = hardwareMap.get(DcMotor.class, IntakeConstants.intakeMotorName);
        this.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.intakeState = IntakeState.STOP;
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }

    public IntakeState getIntakeState() {
        return this.intakeState;
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(intakeState.power);
    }
}
