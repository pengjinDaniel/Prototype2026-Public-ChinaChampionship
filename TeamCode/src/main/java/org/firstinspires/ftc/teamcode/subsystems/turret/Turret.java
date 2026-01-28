package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Util;

public class Turret extends SubsystemBase {
    private TurretState turretState;
    private PIDFController pidfController;
    private DcMotor turretMotor;
    private int ticksSetpoint; // In ticks
    private double normalizedSetpoint; // In the range [-PI, PI]

    public Turret(HardwareMap hardwareMap) {
        this.turretMotor = hardwareMap.get(DcMotor.class, TurretConstants.turretMotorName);
        this.pidfController = new PIDFController(
                TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, TurretConstants.kF
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
                , ticksSetpoint, TurretConstants.alignEpsilon);
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
            if (normalizedSetpoint < TurretConstants.rangeEpsilon - Math.PI + TurretConstants.offset) {
                ticksSetpoint = Units.radiansToTicks(normalizedSetpoint + 2 * Math.PI);
            }
            else if (normalizedSetpoint > -TurretConstants.rangeEpsilon + Math.PI + TurretConstants.offset) {
                ticksSetpoint = Units.radiansToTicks(normalizedSetpoint - 2 * Math.PI);
            }
            else ticksSetpoint = Units.radiansToTicks(normalizedSetpoint);
        }
        else {
            ticksSetpoint = Units.radiansToTicks(normalizedSetpoint);
        }
        if (turretState == TurretState.ACTIVE) {
            turretMotor.setPower(pidfController.calculate(
                    turretMotor.getCurrentPosition(), ticksSetpoint));
        }
    }
}
