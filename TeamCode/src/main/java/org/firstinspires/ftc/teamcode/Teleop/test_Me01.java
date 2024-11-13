package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "test_con")
public class test_Me01 extends LinearOpMode {

    private Servo RightServo;
    private Servo leftservo;
    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor L1;
    private Servo Wrist;
    private DcMotor MotorD;
    private DcMotor L2;
    private Servo neep;

    double Rotate_with_Servo;
    double SpeedLimit;
    boolean Rb_G1Check;
    boolean LB_G1Check;
    boolean Check_count;
    double Arm_0;
    double Wrist_0;
    boolean UP_Wrist;
    boolean LOW_Wrist;
    boolean macro_liftDOWN = false;
    boolean macro_liftUP = false;
    boolean macro_liftspecimen = false;
    /**
     * Describe this function...
     */
    private void do_something() {
    }

    /**
     * Describe this function...
     */
    private void servo_wolfpack(double servo_right) {
        double right_s;

        Rotate_with_Servo = servo_right;
        right_s = servo_right - 0;
        RightServo.setPosition(right_s);
        leftservo.setPosition(1 - right_s);
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
        RightServo = hardwareMap.get(Servo.class, "Right=Servo");
        leftservo = hardwareMap.get(Servo.class, "left=servo");
        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        neep = hardwareMap.get(Servo.class, "neep");

        gamepad1.setLedColor(0.5, 0.1, 0.5, 1000000);
        gamepad2.setLedColor(0.2, 0, 0.6, 1000000);
        // Put initialization blocks here.
        RightServo.setDirection(Servo.Direction.REVERSE);
        leftservo.setDirection(Servo.Direction.REVERSE);
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorB.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        L1.setDirection(DcMotor.Direction.REVERSE);
        Rotate_with_Servo = 0;
        SpeedLimit = 0.6;
        Arm_0 = 0.255;
        Wrist_0 = 1;


        Rb_G1Check = true;
        LB_G1Check = true;
        Check_count = false;
        waitForStart();
        if (opModeIsActive()) {
            servo_wolfpack(Arm_0);
            Wrist.setPosition(Wrist_0);
            while (opModeIsActive()) {
                Test_move2();
                Servo_GamePAD();
                Slide_System(1800, 10);
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Test_move2() {
        float XGame1;
        float YGame1;
        float XGame2;
        float YGame2;
        double SpeedMove;
        double SpeedTurn;
        double drs;
        double dls;
        double A;
        double B;
        double C;
        double D;

        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        XGame1 = gamepad1.left_stick_x * -1;
        YGame1 = gamepad1.left_stick_y * -1;
        XGame2 = gamepad1.right_stick_x * -1;
        YGame2 = gamepad1.right_stick_y * -1;
        if (gamepad2.left_bumper == true && SpeedLimit > 0.32) {
            SpeedLimit += -0.2;
        }
        if (SpeedLimit <= 0.32) {
            SpeedLimit = 0.32;
        }
        if (gamepad2.right_bumper == true && SpeedLimit <= 0.8) {
            SpeedLimit += 0.15;
        }
        if (SpeedLimit >= 1) {
            SpeedLimit = 1;
        }
        SpeedMove = Math.sqrt(Math.pow(XGame1, 2) + Math.pow(YGame1, 2)) * SpeedLimit;
        Rb_G1Check = true;
        LB_G1Check = true;
        SpeedMove = Math.sqrt(Math.pow(XGame1, 2) + Math.pow(YGame1, 2)) * SpeedLimit;
        SpeedTurn = Math.sqrt(Math.pow(XGame2, 2) + Math.pow(YGame2, 2)) * (SpeedLimit + 0.2);
        drs = Math.sin((Math.atan2(YGame1, XGame1) / Math.PI * 180 - 45) / 180 * Math.PI);
        dls = Math.cos((Math.atan2(YGame1, XGame1) / Math.PI * 180 - 45) / 180 * Math.PI);
        if (gamepad1.right_bumper == true && LB_G1Check == true) {
            if (SpeedLimit >= 0.8) {
                SpeedLimit = 0.8;
            } else {
                SpeedLimit = SpeedLimit + 0.2;
            }
            Rb_G1Check = false;
        } else if (gamepad1.left_bumper == true && LB_G1Check == true) {
            if (SpeedLimit <= 0.32) {
                SpeedLimit = 0.32;
            } else {
                SpeedLimit = SpeedLimit - 0.2;
            }
            LB_G1Check = false;
        }
        if (gamepad1.right_bumper == false) {
            sleep(20);
            Rb_G1Check = true;
        } else if (gamepad1.left_bumper == false) {
            sleep(20);
            LB_G1Check = true;
        }
        if (YGame1 < 0) {
            if (dls >= drs) {
                dls = -(dls / drs);
                drs = -1;
            } else {
                drs = -(drs / dls);
                dls = -1;
            }
        } else {
            if (drs <= dls) {
                drs = drs / dls;
                dls = 1;
            } else {
                dls = dls / drs;
                drs = 1;
            }
        }
        A = Math.min(Math.max(drs * SpeedMove, -SpeedLimit), SpeedLimit);
        B = Math.min(Math.max(dls * SpeedMove, -SpeedLimit), SpeedLimit);
        C = Math.min(Math.max(dls * SpeedMove, -SpeedLimit), SpeedLimit);
        D = Math.min(Math.max(drs * SpeedMove, -SpeedLimit), SpeedLimit);
        A += Math.sin((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        B += Math.cos((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        C += Math.sin((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        D += Math.cos((Math.atan2(YGame2, XGame2) / Math.PI * 180 - 45) / 180 * Math.PI) * SpeedTurn;
        MotorA.setPower(A);
        MotorB.setPower(B);
        MotorC.setPower(C);
        MotorD.setPower(D);
    }

    /**
     * Describe this function...
     */
    private void Slide_System(int UP_Degree, int DOWN_Limit) {


        if (gamepad2.dpad_up) {
            macro_liftUP = true;
        } else if (gamepad2.dpad_down) {
            macro_liftDOWN = true;
        }
        if (gamepad2.circle) {
            macro_liftspecimen = true;
        }
        if (macro_liftUP == true) {
            L2.setPower(0.8);
            if (L1.getCurrentPosition() > L2.getCurrentPosition() && L1.getCurrentPosition() < UP_Degree) {
                L1.setPower(0.76);
            } else if (L1.getCurrentPosition() < L2.getCurrentPosition() && L1.getCurrentPosition() < UP_Degree) {
                L1.setPower(0.84);
            } else if (L1.getCurrentPosition() < UP_Degree) {
                L1.setPower(0.8);
            } else {
                L1.setPower(0.05);
                L2.setPower(0.05);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                macro_liftUP = false;
                Check_count = false;
            }
            if (L2.getCurrentPosition() >= 50 && Check_count == false) {
                UP_Wrist = false;
                LOW_Wrist = false;
                Check_count = true;
                servo_wolfpack(0.55);
                Arm_0 = 0.55;
                Wrist.setPosition(0.93);
                Wrist_0 = 0.93;
            }
        } else if (macro_liftDOWN) {
            if (L1.getCurrentPosition() > UP_Degree * 0.8 && L2.getCurrentPosition() > UP_Degree * 0.8) {
                L1.setPower(-0.8);
                L2.setPower(-0.8);
            } else if (L1.getCurrentPosition() > DOWN_Limit && L2.getCurrentPosition() > DOWN_Limit) {
                L1.setPower(-0.8);
                L1.setPower(-0.8);
            } else {
                L1.setPower(0);
                L2.setPower(0);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                servo_wolfpack(0.255);
                Arm_0 = 0.255;
                Wrist.setPosition(1);
                Wrist_0 = 1;
                macro_liftDOWN = false;
                Check_count = false;
            }
            if (L2.getCurrentPosition() <= 1500 && Check_count == false) {
                UP_Wrist = false;
                LOW_Wrist = false;
                Check_count = true;
            }
        } else if (macro_liftspecimen) {
            L2.setPower(0.8);
            if (L1.getCurrentPosition() > L2.getCurrentPosition() && L1.getCurrentPosition() < 480) {
                L1.setPower(0.76);
            } else if (L1.getCurrentPosition() < L2.getCurrentPosition() && L1.getCurrentPosition() < 480) {
                L1.setPower(0.84);
            } else if (L1.getCurrentPosition() < 480) {
                L1.setPower(0.8);
            } else {
                L1.setPower(0.05);
                L2.setPower(0.05);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                macro_liftspecimen = false;
                Check_count = false;
            }
            if (L2.getCurrentPosition() >= 50 && Check_count == false) {
                UP_Wrist = false;
                LOW_Wrist = false;
                Check_count = true;
                servo_wolfpack(0.63);
                Arm_0 = 0.63;
                Wrist.setPosition(1);
                Wrist_0 = 1;
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Servo_GamePAD() {
        telemetry.addData("key", L1.getCurrentPosition());
        telemetry.addData("key", L2.getCurrentPosition());
        telemetry.update();
        if (gamepad2.cross) {
            servo_wolfpack(0.8);
            Wrist.setPosition(0.7);
            Arm_0 = 0.8;
            Wrist_0 = 0.7;
        }
        if (gamepad1.circle) {
            neep.setPosition(0);
        } else if (gamepad1.cross) {
            neep.setPosition(0.4);
        }
        if (gamepad1.triangle) {
            Wrist.setPosition(0.5);
            Wrist_0 = 0.5;
        }
        if (gamepad1.square) {
            Wrist.setPosition(1);
            neep.setPosition(0.3);
            Wrist_0 = 1;
        }
        if (gamepad2.dpad_left) {
            servo_wolfpack(0.27);
            Arm_0 = 0.27;
        }
        if (gamepad2.right_stick_y > 0.05) {
            Arm_0 += -0.07 * Math.abs(gamepad2.right_stick_y);
        } else if (gamepad2.right_stick_y < -0.05) {
            Arm_0 += 0.07 * Math.abs(gamepad2.right_stick_y);
        }
        servo_wolfpack(Arm_0);
        if (gamepad2.triangle) {
            Wrist_0 += -0.1;
        } else if (gamepad2.square) {
            Wrist_0 += 0.1;
        }
        Wrist.setPosition(Wrist_0);
        if (gamepad2.left_stick_y > 0.1 && (L1.getCurrentPosition() + L2.getCurrentPosition()) / 2 > 10) {
            L1.setPower(Math.abs(gamepad2.left_stick_y) * -0.3);
            L2.setPower(Math.abs(gamepad2.left_stick_y) * -0.3);
        } else if (gamepad2.left_stick_y < -0.1 && (L1.getCurrentPosition() + L2.getCurrentPosition()) / 2 < 1750) {
            L1.setPower(Math.abs(gamepad2.left_stick_y) * 0.5);
            L2.setPower(Math.abs(gamepad2.left_stick_y) * 0.5);
        } else {
            L1.setPower(0.02);
            L2.setPower(0.02);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
}