package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "DashTuner")
@Config
public class DashTuner extends OpMode {
    public static String[] motorName = {"", "", "", ""};
    public static String[] servoName = {"", "", "", ""};
    public static double[] motorTarget = new double[4];
    public static double[] servoTarget = new double[4];
    public static int[] slaveTo = {-1, -1, -1, -1};
    public static boolean[] isPositionCloseLoop = new boolean[4];
    public static boolean[] isVelocityCloseLoop = new boolean[4];

    // PIDF parameters as static arrays for Dashboard configuration
    public static double[] kP = {0, 0, 0, 0};
    public static double[] kI = {0, 0, 0, 0};
    public static double[] kD = {0, 0, 0, 0};
    public static double[] kF = {0, 0, 0, 0};

    DcMotorEx[] motors = new DcMotorEx[4];

    Servo[] servos = new Servo[4];

    PIDFController[] pidfControllers = {
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0),
            new PIDFController(0, 0, 0, 0)
    };

    public static String[] colorSensorName = {"", "", "", ""};

    ColorSensor[] colorSensor = new ColorSensor[4];

    DistanceSensor[] distanceSensor = new DistanceSensor[4];

    float hsvValues[] = {0F, 0F, 0F};

    final double SCALE_FACTOR = 255;

    FtcDashboard dashboard;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                motors[i] = hardwareMap.get(DcMotorEx.class, motorName[i]);
                if (isVelocityCloseLoop[i]) {
                    motors[i].setPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(
                                    kP[i],
                                    kI[i],
                                    kD[i],
                                    kF[i]
                            )
                    );
                }
                if (isPositionCloseLoop[i]) {
                    pidfControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
                }
            }
            if (!servoName[i].isEmpty()) {
                servos[i] = hardwareMap.get(Servo.class, servoName[i]);
            }
            if (!colorSensorName[i].isEmpty()) {
                colorSensor[i] = hardwareMap.get(ColorSensor.class, colorSensorName[i]);
                distanceSensor[i] = hardwareMap.get(DistanceSensor.class, colorSensorName[i]);
            }
        }
    }

    @Override
    public void loop() {
        for (int i = 0; i < 4; i++) {
            if (!motorName[i].isEmpty()) {
                if (slaveTo[i] != -1) {
                    if (isVelocityCloseLoop[slaveTo[i]] && !isPositionCloseLoop[slaveTo[i]]) {
                        motors[i].setPIDFCoefficients(
                                DcMotor.RunMode.RUN_USING_ENCODER,
                                new PIDFCoefficients(
                                        kP[slaveTo[i]],
                                        kI[slaveTo[i]],
                                        kD[slaveTo[i]],
                                        kF[slaveTo[i]]
                                )
                        );
                        motors[i].setVelocity(-motorTarget[slaveTo[i]]);
                    }
                }
                else if (isVelocityCloseLoop[i] && !isPositionCloseLoop[i]) {
                    motors[i].setPIDFCoefficients(
                            DcMotor.RunMode.RUN_USING_ENCODER,
                            new PIDFCoefficients(
                                    kP[i],
                                    kI[i],
                                    kD[i],
                                    kF[i]
                            )
                    );

                    motors[i].setVelocity(motorTarget[i]);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("targetVelocity " + i, motorTarget[i]);
                    packet.put("currentVelocity " + i, motors[i].getVelocity());
                    packet.put("currentPower" + i, motors[i].getPower());

                    dashboard.sendTelemetryPacket(packet);
                }
                else if (isPositionCloseLoop[i] && !isVelocityCloseLoop[i]) {
                    pidfControllers[i].setPIDF(kP[i], kI[i], kD[i], kF[i]);
                    double pos = motors[i].getCurrentPosition();

                    motors[i].setPower(pidfControllers[i].calculate(pos, motorTarget[i]));

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("targetPosition " + i, motorTarget[i]);
                    packet.put("currentPosition " + i, pos);

                    dashboard.sendTelemetryPacket(packet);
                }
                else if (!isPositionCloseLoop[i] && !isVelocityCloseLoop[i]) {
                    motors[i].setPower(motorTarget[i]);

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("currentVelocity " + i, motors[i].getVelocity());
                    packet.put("currentPosition " + i, motors[i].getCurrentPosition());

                    dashboard.sendTelemetryPacket(packet);
                }
            }

            if (!servoName[i].isEmpty()) {
                servos[i].setPosition(servoTarget[i]);
            }

            if (!colorSensorName[i].isEmpty()) {
                Color.RGBToHSV((int) (colorSensor[i].red() * SCALE_FACTOR),
                        (int) (colorSensor[i].green() * SCALE_FACTOR),
                        (int) (colorSensor[i].blue() * SCALE_FACTOR),
                        hsvValues);

                TelemetryPacket packet = new TelemetryPacket();

                packet.put("Alpha" + i, colorSensor[i].alpha());
                packet.put("Red" + i, colorSensor[i].red());
                packet.put("Green" + i, colorSensor[i].green());
                packet.put("Blue" + i, colorSensor[i].blue());
                packet.put("Hue" + i, hsvValues[0]);
                packet.put("Distance (cm)" + i, String.format(Locale.US, "%.02f", distanceSensor[i].getDistance(DistanceUnit.CM)));

                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}