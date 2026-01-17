package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;

class PIDF {
    public double kP, kI, kD, kF;

    PIDF(int kP, int kI, int kD, int kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}

@TeleOp(name = "ShooterTest")
@Config
public class ShooterTest extends LinearOpMode {
    public static String motorName1 = "";
    public static String motorName2 = "";
    public static DcMotorEx motor1;
    public static DcMotorEx motor2;
    public static double setpoint = 0;

    public static PIDF[] PIDs = {
            new PIDF(0, 0, 0, 0),
            new PIDF(0, 0, 0, 0)
    };

    @Override
    public void runOpMode() {
        motor1 = hardwareMap.get(DcMotorEx.class, motorName1);
        motor2 = hardwareMap.get(DcMotorEx.class, motorName2);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();
        while (opModeIsActive()) {
            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDs[0].kP, PIDs[0].kI, PIDs[0].kD, PIDs[0].kF));
            motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(PIDs[0].kP, PIDs[0].kI, PIDs[0].kD, PIDs[0].kF));
            motor1.setVelocity(setpoint);
            motor2.setVelocity(setpoint);
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("motor1Vel", motor1.getVelocity());
            packet.put("motor2Vel", motor2.getVelocity());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
