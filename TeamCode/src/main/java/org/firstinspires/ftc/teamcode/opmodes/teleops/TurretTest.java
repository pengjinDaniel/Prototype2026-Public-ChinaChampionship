package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;

@TeleOp(name = "TurretTest")
public class TurretTest extends OpMode {
    public Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        turret.setTurretState(Turret.TurretState.ACTIVE);
        turret.setTurret(0);
    }
}
