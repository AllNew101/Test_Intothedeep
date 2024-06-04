package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;

@TeleOp(name = "RunReady")

public class RunReady extends LinearOpMode {

    private ServoController ControlHub_ServoController;
    private DcMotor MotorA;
    private DcMotor MotorC;
    private DcMotor MotorB;
    private DcMotor MotorD;

    double SpeedLimit;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double speed;
        // TODO: Enter the type for variable named armcount


        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");

        // Put initialization blocks here.
        waitForStart();
        ControlHub_ServoController.pwmEnable();
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        // Put run blocks here.
        speed = 0.6;
        SpeedLimit = 0.4;
        MotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Test_move2();
                telemetry.addData("meca", SpeedLimit);

                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void Test_move2() {
        float XGame1;
        double dls;
        float YGame1;
        double drs;
        float XGame2;
        float YGame2;
        double SpeedMove;
        boolean Rb_G1Check;
        boolean LB_G1Check;
        double SpeedTurn;
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
            SpeedLimit -= 0.02;
        }
        if (SpeedLimit <= 0.32) {
            SpeedLimit = 0.32;
        }
        if (gamepad2.right_bumper == true && SpeedLimit <= 0.8) {
            SpeedLimit += 0.015;
        }
        if (SpeedLimit >= 0.8) {
            SpeedLimit = 0.8;
        }
        SpeedMove = Math.sqrt(Math.pow(XGame1, 2) + Math.pow(YGame1, 2)) * SpeedLimit;
        Rb_G1Check = true;
        LB_G1Check = true;
        SpeedMove = Math.sqrt(Math.pow(XGame1, 2) + Math.pow(YGame1, 2)) * SpeedLimit;
        SpeedTurn = Math.sqrt(Math.pow(XGame2, 2) + Math.pow(YGame2, 2)) * SpeedLimit;
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
}