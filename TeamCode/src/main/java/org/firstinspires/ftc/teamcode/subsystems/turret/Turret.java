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
    private double actualSetpoint; // In the range [-PI - epsilon, PI + epsilon]
    private double normalizedSetPoint; // In the range [-PI, PI]

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        this.pidController = new PIDController(
                TurretConstants.kp, TurretConstants.ki, TurretConstants.kd
        );
        this.turretState = TurretState.INIT;
        normalizedSetPoint = TurretConstants.initPos;
    }

    public enum TurretState {
        INIT,
        ACTIVE;

        TurretState() {}
    }

    public boolean isAligned(){
        return Util.epsilonEqual(turretMotor.getCurrentPosition(),
                normalizedSetPoint, TurretConstants.turretEpsilon);
    }

    /**
    * @param setpoint the setpoint of the turret that will normalize into the range [-PI, PI].
     */
    public void setTurret(double setpoint) {
        this.normalizedSetPoint = Util.adjustRange(setpoint);
    }

    public double getActualPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getNormalizedPosition() {
        return Util.adjustRange(getActualPosition());
    }

    @Override
    public void periodic() {
        if (Math.abs(normalizedSetPoint - getActualPosition()) > Math.PI) {
            if (normalizedSetPoint + Math.PI < TurretConstants.turretEpsilon) {
                actualSetpoint = normalizedSetPoint + 2 * Math.PI;
            }
            else if (normalizedSetPoint - Math.PI > TurretConstants.turretEpsilon) {
                actualSetpoint = normalizedSetPoint - 2 * Math.PI;
            }
        }
        if (turretState == TurretState.ACTIVE) {
            turretMotor.setPower(pidController.calculate(getActualPosition(), actualSetpoint));
        }
    }
}
