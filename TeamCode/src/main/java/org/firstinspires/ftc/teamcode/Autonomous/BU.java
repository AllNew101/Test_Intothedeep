package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BU")
public class BU extends LinearOpMode {

    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor MotorD;

    int Turn_odo;
    int D_odo;
    int x_odo;
    int y_odo;
    int B_odo;
    int C_odo;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");

        // Put initialization blocks here.
        waitForStart();
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if (opModeIsActive()) {
            x_odo = 0;
            y_odo = 0;
            B_odo = 0;
            C_odo = 0;
            D_odo = 0;
            Turn_odo = 0;
            // Put run blocks here.

//            move2(0.3, 0.006, 0, 0.2, 0, 0, 0, -2000, 0);
//            move2(0.2, 0.006, 0, 0.005, 0, 0, 2000, 0, 0);
//            move2(0.2, 0.006, 0, 0.005, 0, 0, 2000, 1000, 0);
            move2(0.8, 0.006, 0, 0.005, 0, 0, 0, 4000, 0);
            move2(0.8, 0.006, 0, 0.005, 0, 0, 10000, 4000, 0);
//            move2(0.6, 0.006, 0, 0.005, 0, 0, 0, 8000, 0);
//            move2(0.3, 0.006, 0, 0.005, 0, 0, 0, 0, 0);
//            move2(0.2, 0.006, 0, 0.005, 0, 0, 0, 0, 0);


//            move2(0.3, 0.006, 0, 0.2, 0, 0, 0, -4000, 90);
//            move2(0.3, 0.006, 0, 0.2, 0, 0, 2000, 0, 90);
//            move2(0.3, 0.006, 0, 0.2, 0, 0, 2000, -2000, 90);
//            move2(0.15, 0.006, 0, 0.2, 0, 0, 0, 0, 0);
            sleep(5000);
        }
    }

    /**
     * Describe this function...
     */
    private void move2(double Speed, double kp, double kd, double kp_XY, double ki_XY, double kd_XY, int y_positions, int x_positions, int Turn) {
        double movement;
        int error_sum;
        ElapsedTime myElapsedTime;
        double pidXY = 0;
        double lastTime = 0;
        double degree = 0;
        int previous_odoturn = 0;
        int error_sum_XY = 0;
        int errorRate_XY = 0;
        int error_turn = 0;
        double errorRate = 0;
        double dt = 0;
        double error_XY = 0;
        double previous_odoXY = 0;
        double last_turn = 0;
        double str = 0;
        double forw = 0;
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        x_positions = x_positions * -1;
        lastTime = 0;
        previous_odoturn = 0;
        myElapsedTime = new ElapsedTime();
        error_sum = 0;
        error_sum_XY = 0;
        errorRate_XY = 0;
        myElapsedTime.reset();
        while (myElapsedTime.seconds()< 30) {
            telemetry.addData("x", x_odo);
            telemetry.addData("y", y_odo);
            telemetry.addData("turn", Turn_odo);
            telemetry.addData("b_current", MotorB.getCurrentPosition());
            telemetry.addData("c_current", MotorC.getCurrentPosition());
            telemetry.addData("TurnBrush",(Turn_odo / 180.0 * Math.PI));
            telemetry.addData("sin_Turn", Math.sin((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("cos_Turn", Math.cos((Turn_odo / 180.0 * Math.PI)));

            telemetry.update();


            Turn_odo = ((-MotorB.getCurrentPosition()) - (-MotorC.getCurrentPosition()))/ 54;
            forw = ((-MotorB.getCurrentPosition() - B_odo) + (-MotorC.getCurrentPosition() - C_odo)) / 2.0;
            str = ((-MotorD.getCurrentPosition() - D_odo) - (33.0 * (Turn_odo-last_turn)));
            x_odo += (forw * Math.sin(Turn_odo / 180.0 * Math.PI)) + ((str * Math.cos(Turn_odo / 180.0 * Math.PI)));
            y_odo += (forw * Math.cos(Turn_odo / 180.0 * Math.PI)) - ((str * Math.sin(Turn_odo / 180.0 * Math.PI)));
            last_turn = Turn_odo ;
            B_odo = -MotorB.getCurrentPosition();
            C_odo = -MotorC.getCurrentPosition();
            D_odo = -MotorD.getCurrentPosition();

            error_turn = Turn - Turn_odo;
//               errorRate = (error_turn - previous_odoturn) / dt;
//               dt = myElapsedTime.milliseconds() - lastTime;
//               error_sum += error_turn * dt;
            movement = error_turn * kp + errorRate * kd + 0;
            degree = ((Math.atan2(y_positions - y_odo, x_positions - x_odo) / Math.PI * 180.0) - 45.0) + Turn_odo;
            if (y_positions - y_odo >= 100 || y_positions - y_odo <= -100 || x_positions - x_odo >= 100 || x_positions - x_odo <= -100 || Turn_odo - Turn >= 3 || Turn_odo - Turn <= -3) {
//                 error_XY = Math.sqrt(Math.pow(y_positions, 2) + Math.pow(x_positions, 2)) - Math.sqrt(Math.pow(y_odo, 2) + Math.pow(x_odo, 2));
//                 if (error_XY <= 2) {
//                   error_sum_XY += error_XY * dt;
//                   errorRate_XY = (int) ((error_XY - previous_odoXY) / dt);
//                 }
//                 pidXY = error_XY * kp_XY + errorRate_XY * kd_XY + error_sum_XY * ki_XY;
                MotorA.setPower(Math.min(Math.max(Math.sin(degree / 180.0 * Math.PI) * Speed, -Speed), Speed) + movement);
                MotorB.setPower(Math.min(Math.max(Math.cos(degree / 180.0 * Math.PI) * Speed, -Speed), Speed) - movement);
                MotorC.setPower(Math.min(Math.max(Math.cos(degree / 180.0 * Math.PI) * Speed, -Speed), Speed) + movement);
                MotorD.setPower(Math.min(Math.max(Math.sin(degree / 180.0 * Math.PI) * Speed, -Speed), Speed) - movement);
            } else {
                MotorA.setPower(0);
                MotorB.setPower(0);
                MotorC.setPower(0);
                MotorD.setPower(0);
                MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                myElapsedTime.reset();
                break;
            }
            previous_odoturn = error_turn;
            lastTime = myElapsedTime.milliseconds();
            previous_odoXY = error_XY;

            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private double Connvert_ticks_2_centimeter(double ticks) {
        return (ticks / 1248) * Math.PI * 4.8;
    }
}
