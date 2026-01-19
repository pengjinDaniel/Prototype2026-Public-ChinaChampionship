package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

public class Turret extends SubsystemBase {
    private TurretState turretState;
    private PIDController pidController;
    private DcMotor turretMotor;
    private int ticksSetpoint; // In ticks
    private double normalizedSetpoint; // In the range [-PI, PI]

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        this.pidController = new PIDController(
                TurretConstants.kp, TurretConstants.ki, TurretConstants.kd
        );
        this.turretState = TurretState.INIT;
        normalizedSetpoint = TurretConstants.initPos;
    }

    public enum TurretState {
        INIT,
        ACTIVE;

        TurretState() {}
    }

    public boolean isAligned(){
        return Util.epsilonEqual(turretMotor.getCurrentPosition()
                , ticksSetpoint, TurretConstants.turretEpsilon);
    }

    public void setTurretState(TurretState turretState) {
        this.turretState = turretState;
    }

    /**
    * @param setpoint the setpoint of the turret that will normalize into the range [-PI, PI].
     */
    public void setTurret(double setpoint) {
        this.normalizedSetpoint = Util.adjustRange(setpoint);
    }

    @Override
    public void periodic() {
        if (Math.abs(normalizedSetpoint - Units.ticksToRadians(
                turretMotor.getCurrentPosition())) > Math.PI) {
            if (normalizedSetpoint + Math.PI < TurretConstants.turretEpsilon) {
                ticksSetpoint = Units.radiansToTicks(normalizedSetpoint + 2 * Math.PI);
            }
            else if (normalizedSetpoint - Math.PI > TurretConstants.turretEpsilon) {
                ticksSetpoint = Units.radiansToTicks(normalizedSetpoint - 2 * Math.PI);
            }
        }
        if (turretState == TurretState.ACTIVE) {
            turretMotor.setPower(pidController.calculate(
                    turretMotor.getCurrentPosition(), ticksSetpoint));
        }
    }
}
