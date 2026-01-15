package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Util;

public class Turret extends SubsystemBase {
    private TurretState turretState;
    private PIDController pidController;
    private DcMotor turretMotor;
    private double setpoint;

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        this.pidController = new PIDController(
                TurretConstants.kp, TurretConstants.ki, TurretConstants.kd
        );
        this.turretState = TurretState.INIT;
        setpoint = TurretConstants.initPos;
    }

    public enum TurretState {
        INIT,
        ACTIVE;

        TurretState() {}
    }

    public boolean isAligned(){
        return Util.epsilonEqual(turretMotor.getCurrentPosition(),
                setpoint, TurretConstants.turretEpsilon);
    }

    public void setTurret(double setpoint) {
        this.setpoint = setpoint;
    }

    @Override
    public void periodic() {
        if (!isAligned()) {
            turretMotor.setPower(pidController.calculate(turretMotor.getCurrentPosition(), setpoint));
        }

    }
}
