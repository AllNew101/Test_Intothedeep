package org.firstinspires.ftc.teamcode.Teleop;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.acmerobotics.dashboard.config.Config;
@Disabled
@TeleOp(name = "test_slide")
@Config
public class test_slide extends LinearOpMode {
    private DcMotor MotorA;
    public static int second = 500;
    public static int second_wait = 1000;
    public static double power = 0.5;
    public static boolean Using_encoder = false;
    public static int degree = 1000;
    @Override
    public void runOpMode() throws InterruptedException {
        MotorA = hardwareMap.get(DcMotor.class,"MotorA");
        waitForStart();
        while (true){
        if (gamepad1.y == true){
        if (Using_encoder == true){
            while (true){
                if (Math.abs(MotorA.getCurrentPosition()) < degree){
                    MotorA.setPower(power * -1);
                }
                else{
                    MotorA.setPower(0);
                }
            }
        }
        else {
            MotorA.setPower(power * -1);
            sleep(second);
            MotorA.setPower(0);
        }
        MotorA.setPower(0);
        sleep(second_wait);
        MotorA.setPower(power);
        if (Using_encoder == true){
            while (true){
                if (Math.abs(MotorA.getCurrentPosition())> 10){
                    MotorA.setPower(power);
                }
                else{
                    MotorA.setPower(0);
                }
            }
        }
        else {
            MotorA.setPower(power);
            sleep(second);
            MotorA.setPower(0);
        }
        MotorA.setPower(0);
    }}}
}
