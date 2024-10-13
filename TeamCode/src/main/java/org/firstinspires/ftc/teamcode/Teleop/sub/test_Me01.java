package org.firstinspires.ftc.teamcode.Teleop.sub;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "test_Sub")
public class test_Me01 extends LinearOpMode {

    private Servo RightServo;
    private Servo leftservo;
    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor L1;
    private DcMotor L2;
    private Servo Karn;
    private DcMotor MotorD;

    double SpeedLimit;
    boolean Rb_G1Check;
    boolean LB_G1Check;



    /**
     * Describe this function...
     */
    private void servo_wolfpack(double servo_right) {
        double right_s;

        right_s = servo_right - 0;
        RightServo.setPosition(right_s);
        leftservo.setPosition(0.45 - right_s);
    }

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        long Time;

        RightServo = hardwareMap.get(Servo.class, "Right=Servo");
        leftservo = hardwareMap.get(Servo.class, "left=servo");
        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        Karn = hardwareMap.get(Servo.class, "Karn");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");

        // Put initialization blocks here.
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorB.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        L1.setDirection(DcMotor.Direction.REVERSE);
        SpeedLimit = 0.6;
        Rb_G1Check = true;
        LB_G1Check = true;
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                if (gamepad1.y == true) {
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    Time = System.currentTimeMillis();
                    while (L1.getCurrentPosition() < 1700 || L2.getCurrentPosition() < 1700) {
                        if (L1.getCurrentPosition() > L2.getCurrentPosition() && L1.getCurrentPosition() < 1700) {
                            L1.setPower(0.95);
                        } else if (L1.getCurrentPosition() < 1700) {
                            L1.setPower(1);
                        } else {
                            L1.setPower(0.01);
                            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                        if (L2.getCurrentPosition() > L1.getCurrentPosition() && L2.getCurrentPosition() < 1700) {
                            L2.setPower(0.95);
                        } else if (L2.getCurrentPosition() < 1700) {
                            L2.setPower(1);
                        } else {
                            L2.setPower(0.01);
                            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                        if (L2.getCurrentPosition() >= 50) {
                            servo_wolfpack(0.42);
                            Karn.setPosition(0.35);
                        }
                    }
                    // Get the current time in milliseconds. The value returned represents
                    // the number of milliseconds since midnight, January 1, 1970 UTC.
                    telemetry.addData("key", System.currentTimeMillis() - Time);
                    telemetry.update();
                } else if (gamepad1.x == true) {
                    while (L1.getCurrentPosition() > 10 || L2.getCurrentPosition() > 10) {
                        if (L1.getCurrentPosition() > 10) {
                            L1.setPower(-0.9);
                        } else {
                            L1.setPower(0);
                            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                        if (L2.getCurrentPosition() > 10) {
                            L2.setPower(-0.9);
                        } else {
                            L2.setPower(0);
                            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        }
                        if (L2.getCurrentPosition() <= 1600) {
                            servo_wolfpack(0);
                            Karn.setPosition(0.65);
                        }
                    }
                }
            }
        }
    }
}
