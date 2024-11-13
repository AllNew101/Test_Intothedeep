package org.firstinspires.ftc.teamcode.Autonomous.planB;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name = "KanGood")
public class KanGood extends LinearOpMode {

    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor MotorD;

    int Turn_odo;
    int Delta_Turn_odo;
    int D_odo;
    int x_odo;
    int y_odo;
    int B_odo;
    int C_odo;
    double last_turn = 0;


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



              move2(0.35, 0.08, 0.25,0.008, 0.01,0.05, 0, -10000, 0);
              move2(0.35, 0.08, 0.25,0.008, 0.01,0.05, 5000, -10000, 90);
              move2(0.35, 0.08, 0.25,0.008, 0.01,0.05, -5000, -10000, 90);
              move2(0.35, 0.08, 0.25,0.008, 0.01,0.05, 0, 0, 0);






            sleep(5000);
        }
    }

    /**
     * Describe this function...
     */
    private void move2(double Speed, double kp_Turn, double kd_Turn, double kp_XY, double ki_XY,  double kd_XY, int y_positions, int x_positions, int Turn) {
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
        double str = 0;
        double forw = 0;
        double x_degree;
        double y_degree;
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
        while (myElapsedTime.seconds() < 30) {
            telemetry.addData("x", x_odo);
            telemetry.addData("y", y_odo);
            telemetry.addData("turn", Turn_odo);
            telemetry.addData("b_current", MotorB.getCurrentPosition());
            telemetry.addData("c_current", MotorC.getCurrentPosition());
            telemetry.addData("TurnBrush",(Turn_odo / 180.0 * Math.PI));
            telemetry.addData("sin_Turn", Math.sin((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("cos_Turn", Math.cos((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("Delta_Turn",Delta_Turn_odo);
            telemetry.addData("Degree", degree);




            Turn_odo = ((-MotorB.getCurrentPosition()) - (-MotorC.getCurrentPosition()))/ 54;
            Delta_Turn_odo = Turn_odo - (int) last_turn;//((-MotorB.getCurrentPosition() - B_odo) - (-MotorC.getCurrentPosition() - C_odo)) / 54;
            forw = ((-MotorB.getCurrentPosition() - B_odo) + (-MotorC.getCurrentPosition() - C_odo)) / 2.0;
            str = ((-MotorD.getCurrentPosition() - D_odo) - (33.0 * (Delta_Turn_odo)));
            x_odo += (forw * Math.cos(Turn_odo / 180.0 * Math.PI)) + ((str * Math.sin(Turn_odo / 180.0 * Math.PI)));
            y_odo += ((str * Math.cos(Turn_odo / 180.0 * Math.PI))) - (forw * Math.sin(Turn_odo / 180.0 * Math.PI));

            B_odo = -MotorB.getCurrentPosition();
            C_odo = -MotorC.getCurrentPosition();
            D_odo = -MotorD.getCurrentPosition();

            error_turn = Turn - Turn_odo;
            errorRate = (error_turn - previous_odoturn) / dt;
            dt = myElapsedTime.milliseconds() - lastTime;
            movement = error_turn * kp_Turn + errorRate * kd_Turn ;
          degree = Math.atan2(x_positions - x_odo, y_positions - y_odo)  - (Turn_odo / 180.0 * Math.PI) ;
            x_degree = ((x_positions - x_odo) * Math.cos(Turn_odo)) - ((y_positions - y_odo) * Math.sin(Turn_odo));
            y_degree = ((y_positions - y_odo) * Math.cos(Turn_odo) + (x_positions - x_odo) * Math.sin(Turn_odo));

          //y_positions - y_odo >= 100 || y_positions - y_odo <= -100 || x_positions - x_odo >= 100 || x_positions - x_odo <= -100
//          degree = (Math.atan2(x_degree,y_degree) - Math.PI/4.0);
            degree = (degree - Math.PI/4.0);
            if (Math.abs(Math.sqrt(Math.pow(y_positions - y_odo,2) + Math.pow(x_positions - x_odo,2))) >= 200 || Math.abs(Turn - Turn_odo) >= 3) {
                 error_XY = Math.sqrt(Math.pow(y_positions, 2) + Math.pow(x_positions, 2)) - Math.sqrt(Math.pow(y_odo, 2) + Math.pow(x_odo, 2));
                errorRate_XY = (int) ((error_XY - previous_odoXY) / dt);
                if (Math.abs(error_XY) <= 100) {
                   error_sum_XY += error_XY * dt;}
                pidXY = error_XY * kp_XY + errorRate_XY * kd_XY + error_sum_XY * ki_XY;


                MotorA.setPower(Math.min(Math.max(Math.sin(degree) * Speed + movement, -Speed), Speed) );
                MotorB.setPower(Math.min(Math.max(Math.cos(degree) * Speed - movement, -Speed), Speed) );
                MotorC.setPower(Math.min(Math.max(Math.cos(degree) * Speed + movement, -Speed), Speed) );
                MotorD.setPower(Math.min(Math.max(Math.sin(degree) * Speed - movement, -Speed), Speed) );




                telemetry.addData("Turn done?",Math.abs(Turn - Turn_odo) < 3);
                telemetry.addData("Position done?",Math.abs(Math.sqrt(Math.pow(y_positions - y_odo,2) + Math.pow(x_positions - x_odo,2))) < 100);
                telemetry.update();
            }
            else {
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
            last_turn = Turn_odo ;
        }
    }

    /**
     * Describe this function...
     */
    private double Connvert_ticks_2_centimeter(double ticks) {
        return (ticks / 1248) * Math.PI * 4.8;
    }
}
