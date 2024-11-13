package org.firstinspires.ftc.teamcode.Autonomous.planB;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Config
@Autonomous(name = "AutoBlueBu_Sc")
public class PlanC_inCM extends LinearOpMode {

    public static double con = 5;
    public static double kd = 11;
    public static double kp = 0.15;
    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor MotorD;
    private Servo RightServo;
    private Servo leftservo;

    private DcMotor L1;
    private Servo Wrist;

    private DcMotor L2;
    private Servo neep;
    double Turn_odo;
    double Delta_Turn_odo;
    double D_odo;
    double x_odo;
    double y_odo;
    double B_odo;
    double C_odo;
    double last_turn = 0;
    double Rotate_with_Servo;

    private DistanceSensor Ch;

    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        MotorA = hardwareMap.get(DcMotor.class, "MotorA");
        MotorB = hardwareMap.get(DcMotor.class, "MotorB");
        MotorC = hardwareMap.get(DcMotor.class, "MotorC");
        MotorD = hardwareMap.get(DcMotor.class, "MotorD");
        RightServo = hardwareMap.get(Servo.class, "Right=Servo");
        leftservo = hardwareMap.get(Servo.class, "left=servo");
        L1 = hardwareMap.get(DcMotor.class, "L1");
        Wrist = hardwareMap.get(Servo.class, "Wrist");
        L2 = hardwareMap.get(DcMotor.class, "L2");
        neep = hardwareMap.get(Servo.class, "neep");
        Ch = hardwareMap.get(DistanceSensor.class, "Ch");

        // Put initialization blocks here.
        waitForStart();
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setDirection(DcMotor.Direction.REVERSE);
        MotorB.setDirection(DcMotor.Direction.REVERSE);
        MotorC.setDirection(DcMotor.Direction.REVERSE);
        L1.setDirection(DcMotor.Direction.REVERSE);
        RightServo.setDirection(Servo.Direction.REVERSE);
        leftservo.setDirection(Servo.Direction.REVERSE);
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
//            move2(0.6,kd,kp,0,0,90);

            neep.setPosition(0.1);


            move2(0.4,0.4,0.4,0.4,kd,kp,10.0,0);
//            Wrist.setPosition(0.93);
//            sleep(600);
//            up_sp(950);
//            sleep(300);

//            L1.setPower(-0.7);
//            L2.setPower(-0.7);
//            sleep(400);
//            L1.setPower(0);
//            L2.setPower(0);
//            neep.setPosition(0.4);
//            servo_wolfpack(0.7);
//            sleep(300);
//            L1.setPower(-0.6);
//            L2.setPower(-0.6);
//            sleep(300);
//            L1.setPower(0);
//            L2.setPower(0);
//            neep.setPosition(0.4);
//
//            move2(-0.4,-0.4,-0.4,-0.4,kd,kp,160.0,0);
//            neep.setPosition(0.1);
//            move2(-0.6,0.6,0.6,-0.6,kd,kp,1500.0,0);
//            move_dis(-0.4,0.4,0.4,-0.4,kd,kp,0,53.5);
//            MotorA.setPower(-0.4);
//            MotorB.setPower(-0.4);
//            MotorC.setPower(-0.4);
//            MotorD.setPower(-0.4);
//            sleep(1200);
//            MotorA.setPower(0);
//            MotorB.setPower(0);
//            MotorC.setPower(0);
//            MotorD.setPower(0);
//            sleep(300);
//
//            move2(0.4,0.4,0.4,0.4,kd,kp,730.0,0);
//            move_dis(-0.4,0.4,0.4,-0.4,kd,kp,0,53.5);
//
//
//            servo_wolfpack(0.3);
//            sleep(320);
//            Wrist.setPosition(0.5);
//            neep.setPosition(0.4);
//            sleep(500);
//            servo_wolfpack(0);
//            sleep(300);
//            Wrist.setPosition(0.93);
//            sleep(600);
//            neep.setPosition(0.1);
//            sleep(900);
//            macrolang(760);
//            sleep(300);
//            move2(-0.6,-0.6,-0.6,-0.6,kd,kp,100.0,0);
//            sleep(100);
//
//
//            move2(0,0,0,0,kd,kp,0.0,-48);
//
//            move2(-0.4,-0.4,-0.4,-0.4,kd,kp,530.0,-48);
//            neep.setPosition(0.4);
//            sleep(800);
//            Wrist.setPosition(1);
//            sleep(300);
//            move2(0.4,0.4,0.4,0.4,kd,kp,560.0,-48);
//            servo_wolfpack(0.4);
//            sleep(400);
//
//            L1.setPower(-0.5);
//            L2.setPower(-0.5);
//            sleep(400);
//            L1.setPower(0);
//            L2.setPower(0);
//
//            move2(0.4,0.4,0.4,0.4,kd,kp,0,0);
//            move2(0.4,0.4,0.4,0.4,kd,kp,60,0);
//            move_dis(-0.4,0.4,0.4,-0.4,kd,kp,2,25);
//
////            servo_wolfpack(0.4);
////            sleep(320);
//            Wrist.setPosition(0.5);
//            neep.setPosition(0.4);
//            sleep(200);
//            servo_wolfpack(0.12);
//            sleep(300);
//            Wrist.setPosition(1);
//            sleep(600);
//            neep.setPosition(0.1);
//            sleep(600);
//            macrolang(760);
//
//            move2(-0.6,-0.6,-0.6,-0.6,kd,kp,300.0,0);
//            sleep(300);
//
//            move2(0,0,0,0,kd,kp,0.0,-42);
//            move2(0,0,0,0,kd,kp,0.0,-42);
//
//            move2(-0.4,-0.4,-0.4,-0.4,kd,kp,0.0,-42);
//            move2(-0.4,-0.4,-0.4,-0.4,kd,kp,100.0,-42);
//            neep.setPosition(0.4);
//            sleep(800);
//            Wrist.setPosition(1);
//            sleep(400);
//            move2(0.4,0.4,0.4,0.4,kd,kp,300.0,-42);
//            servo_wolfpack(0.15);
//            sleep(300);
//            L1.setPower(-0.65);
//            L2.setPower(-0.65);
//            sleep(380);
//            L1.setPower(0);
//            L2.setPower(0);
//            move2(0.4,0.4,0.4,0.4,kd,kp,0,-30);
//            servo_wolfpack(0.3);
//
//            move2(0.4,0.4,0.4,0.4,kd,kp,2000,-80);
//            servo_wolfpack(0.15);
//            sleep(200);



        }
    }
    private void servo_wolfpack(double servo_right) {
        double right_s;

        Rotate_with_Servo = servo_right;
        right_s = servo_right - 0;
        RightServo.setPosition(right_s);
        leftservo.setPosition(1 - right_s);
    }
    public void up_sp(int UP) {

        ElapsedTime myElapsedTime;
        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();
        while (myElapsedTime.seconds() < 10){
            L2.setPower(0.6);
            if (L1.getCurrentPosition() > L2.getCurrentPosition() && L1.getCurrentPosition() < UP) {
                L1.setPower(0.56);
            } else if (L1.getCurrentPosition() < L2.getCurrentPosition() && L1.getCurrentPosition() < UP) {
                L1.setPower(0.64);
            } else if (L1.getCurrentPosition() < UP) {
                L1.setPower(0.6);
            } else {
                L1.setPower(0.05);
                L2.setPower(0.05);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;

            }
            if (L2.getCurrentPosition() >= 50 ) {

                servo_wolfpack(0.4);
                Wrist.setPosition(0.93);

            }
        }}
    public void down_sp(int UP,int Down) {
        ElapsedTime myElapsedTime;
            myElapsedTime = new ElapsedTime();
            myElapsedTime.reset();

            while (myElapsedTime.seconds() < 10){
            if (L1.getCurrentPosition() > UP && L2.getCurrentPosition() > UP) {
                L1.setPower(-0.7);
                L2.setPower(-0.7);
            } else if (L1.getCurrentPosition() > Down && L2.getCurrentPosition() > Down) {
                L1.setPower(-0.7);
                L1.setPower(-0.7);
            } else {
                L1.setPower(0);
                L2.setPower(0);
                L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
                //servo_wolfpack(0.215);
                //Wrist.setPosition(1);

            }
        }}
    public void macrolang (int UP_Degree){
        ElapsedTime myElapsedTime;
        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();

        while (myElapsedTime.seconds() < 10){


        L2.setPower(0.6);
        if (L1.getCurrentPosition() > L2.getCurrentPosition() && L1.getCurrentPosition() < UP_Degree) {
            L1.setPower(0.56);
        } else if (L1.getCurrentPosition() < L2.getCurrentPosition() && L1.getCurrentPosition() < UP_Degree) {
            L1.setPower(0.64);
        } else if (L1.getCurrentPosition() < UP_Degree) {
            L1.setPower(0.6);
        } else {
            L1.setPower(0.05);
            L2.setPower(0.05);
            L1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            L2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            servo_wolfpack(0.6);
            Wrist.setPosition(0.75);
            break;
    }}}
    public void move2(double SpeedA,double SpeedB,double SpeedC,double SpeedD, double kp_Turn, double kd_Turn, double CM, double Turn){
        double movement;
        ElapsedTime myElapsedTime;
        double pidXY = 0;
        double lastTime = 0;
        double degree = 0;
        double previous_odoturn = 0;
        double error_sum_XY = 0;
        double errorRate_XY = 0;
        double error_turn = 0;
        double errorRate = 0;
        double dt = 0;
        double error_XY = 0;
        double previous_odoXY = 0;
        double str = 0;
        double forw = 0;
        double x_degree;
        double y_degree;
        double slowX = 0;
        double slowY = 0;
        movement = 0;
        double t_position_x = x_odo;
        double t_position_y = y_odo;
        double encoderCM;
        int check_stop = 0;
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();


        while (myElapsedTime.seconds() < 30) {

            telemetry.addData("x", x_odo);
            telemetry.addData("y", y_odo);
            telemetry.addData("turn", Turn_odo);
            telemetry.addData("b_current", B_odo);
            telemetry.addData("c_current", C_odo);
            telemetry.addData("d_current", D_odo);
            telemetry.addData("forw", forw);
            telemetry.addData("str", str);
            telemetry.addData("TurnBrush", (Turn_odo / 180.0 * Math.PI));
            telemetry.addData("sin_Turn", Math.sin((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("cos_Turn", Math.cos((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("Delta_Turn", Delta_Turn_odo);
            telemetry.addData("Degree", degree);
            telemetry.addData("Movement", movement);
            telemetry.addData("KP", kp);
            telemetry.addData("KD", kd);
            telemetry.addData("error_turn", error_turn);

            telemetry.update();


            Turn_odo = (double) ((-MotorB.getCurrentPosition()) - (MotorC.getCurrentPosition())) / 72.0;

            error_turn = Turn_odo - Turn;
            errorRate = (error_turn - previous_odoturn) ;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn);
            encoderCM = (Math.abs(MotorA.getCurrentPosition()) / 537.6) * 2 * Math.PI * 4.8;

            if (encoderCM < CM * 0.8){
                MotorA.setPower(SpeedA);
                MotorB.setPower(SpeedB);
                MotorC.setPower(SpeedC);
                MotorD.setPower(SpeedD);
            }
            else if (encoderCM < CM){
                MotorA.setPower(SpeedA * 0.4);
                MotorB.setPower(SpeedB * 0.4);
                MotorC.setPower(SpeedC * 0.4);
                MotorD.setPower(SpeedD * 0.4);
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


                check_stop ++;
            }

            if (Math.abs(error_turn) >= 3){
                telemetry.addData("Check", "12");
                MotorA.setPower(Math.min(Math.max((+ (movement)), -0.25), 0.25));
                MotorB.setPower(Math.min(Math.max((- (movement)), -0.25), 0.25));
                MotorC.setPower(Math.min(Math.max((+ (movement)), -0.25), 0.25));
                MotorD.setPower(Math.min(Math.max((- (movement)), -0.25), 0.25));
                check_stop ++;
            }
            if (check_stop == 2){
                myElapsedTime.reset();
                break;
            }

            previous_odoturn = error_turn;
            last_turn = Turn_odo;
        }
    }
    public void move_dis(double SpeedA,double SpeedB,double SpeedC,double SpeedD, double kp_Turn, double kd_Turn,  int Turn ,double dis){
        double movement;
        ElapsedTime myElapsedTime;
        double pidXY = 0;
        double lastTime = 0;
        double degree = 0;
        double previous_odoturn = 0;
        double error_sum_XY = 0;
        double errorRate_XY = 0;
        double error_turn = 0;
        double errorRate = 0;
        double dt = 0;
        double error_XY = 0;
        double previous_odoXY = 0;
        double str = 0;
        double forw = 0;
        double x_degree;
        double y_degree;
        double slowX = 0;
        double slowY = 0;
        movement = 0;
        double t_position_x = x_odo;
        double t_position_y = y_odo;
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();

        while (myElapsedTime.seconds() < 30) {

            telemetry.addData("x", x_odo);
            telemetry.addData("y", y_odo);
            telemetry.addData("turn", Turn_odo);
            telemetry.addData("b_current", B_odo);
            telemetry.addData("c_current", C_odo);
            telemetry.addData("d_current", D_odo);
            telemetry.addData("forw", forw);
            telemetry.addData("str", str);
            telemetry.addData("TurnBrush", (Turn_odo / 180.0 * Math.PI));
            telemetry.addData("sin_Turn", Math.sin((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("cos_Turn", Math.cos((Turn_odo / 180.0 * Math.PI)));
            telemetry.addData("Delta_Turn", Delta_Turn_odo);
            telemetry.addData("Degree", degree);
            telemetry.addData("Movement", movement);
            telemetry.addData("KP", kp);
            telemetry.addData("KD", kd);
            telemetry.addData("error_turn", error_turn);

            telemetry.update();


            Turn_odo = (double) ((-MotorB.getCurrentPosition()) - (MotorC.getCurrentPosition())) / 72.0;

            error_turn = Turn_odo - Turn;
            errorRate = (error_turn - previous_odoturn) ;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn);
            if (Ch.getDistance(DistanceUnit.CM) > dis){
                MotorA.setPower(SpeedA);
                MotorB.setPower(SpeedB);
                MotorC.setPower(SpeedC);
                MotorD.setPower(SpeedD);
            }
            else if (Math.abs(error_turn) >= 2){
                telemetry.addData("Check", "12");
                MotorA.setPower(Math.min(Math.max((+ (movement)), -0.2), 0.2));
                MotorB.setPower(Math.min(Math.max((- (movement)), -0.2), 0.2));
                MotorC.setPower(Math.min(Math.max((+ (movement)), -0.2), 0.2));
                MotorD.setPower(Math.min(Math.max((- (movement)), -0.2), 0.2));

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
            last_turn = Turn_odo;
        }
    }


}



