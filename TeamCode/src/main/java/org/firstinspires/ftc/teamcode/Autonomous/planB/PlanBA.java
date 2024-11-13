package org.firstinspires.ftc.teamcode.Autonomous.planB;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@Config
@Autonomous(name = "PlanBA")
public class PlanBA extends LinearOpMode {

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
            Wrist.setPosition(1);
            move2(0.4,0.4,0.4,0.4,kd,kp,1020.0,0);
            up_sp(890);


            L1.setPower(-0.5);
            L2.setPower(-0.5);
            sleep(350);
            L1.setPower(0);
            L2.setPower(0);
            neep.setPosition(0.4);
            servo_wolfpack(0.7);
            L1.setPower(-0.5);
            L2.setPower(-0.5);
            sleep(350);
            L1.setPower(0);
            L2.setPower(0);
            neep.setPosition(0.4);

            move2(0.4,-0.5,-0.5,0.4,kd,kp,1780.0,0);
            move_dis(0.4,-0.4,-0.4,0.4,kd,kp,0,51.7);

            servo_wolfpack(0.4);
            sleep(350);
            servo_wolfpack(0.12);
            Wrist.setPosition(0.93);
            sleep(1500);
            neep.setPosition(0.1);
            sleep(600);
            servo_wolfpack(0.8);
            Wrist.setPosition(0.7);
            move2(-0.2,-0.6,-0.6,-0.2,kd,kp,300.0,0);

            sleep(300);
            neep.setPosition(0.4);
            move2(0.6,0,0,0.6,kd,kp,800.0,0);
            move_dis(0.4,-0.4,-0.4,0.4,kd,kp,0,26);
            move2(0.4,0.4,0.4,0.4,kd,kp,80.0,0);
            servo_wolfpack(0.4);
            sleep(400);
            servo_wolfpack(0.12);
            Wrist.setPosition(0.93);
            sleep(1400);
            neep.setPosition(0.1);
            sleep(1000);
            servo_wolfpack(0.8);
            Wrist.setPosition(0.7);
            servo_wolfpack(0.8);
            Wrist.setPosition(0.7);
            move2(-0.2,-0.6,-0.6,-0.2,kd,kp,300.0,0);

            sleep(300);
            neep.setPosition(0.4);
            sleep(1000);
            move2(-0.4,0.4,0.4,-0.4,kd,kp,500.0,0);
            move2(0.4,0.4,0.4,0.4,kd,kp,2000.0,0);
            move2(0.4,-0.4,-0.4,0.4,kd,kp,370.0,0);
            move2(0.4,-0.4,-0.4,0.4,kd,kp,0,-3);
            move2(-0.4,-0.4,-0.4,-0.4,kd,kp,1700.0,0);
//            move2(0.6,kd,kp,13500,3000,0);
//            sleep(300);
//            neep.setPosition(0.4);
//            sleep(1000);
//            move2(0.6,kd,kp,13500,8000,0);
//            move2(0.6,kd,kp,14000,8000,0);
//            move2(0.6,kd,kp,14000,1000,0);
//        move_bazier(0.6,kd,kp,10700,4450,0,0,0,0,0);

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

                servo_wolfpack(0.27);
                Wrist.setPosition(0.93);

            }
        }}
    public void down_sp(int UP,int Down) {
        ElapsedTime myElapsedTime;
        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();

        while (myElapsedTime.seconds() < 10){
            if (L1.getCurrentPosition() > UP && L2.getCurrentPosition() > UP) {
                L1.setPower(-0.6);
                L2.setPower(-0.6);
            } else if (L1.getCurrentPosition() > Down && L2.getCurrentPosition() > Down) {
                L1.setPower(-0.2);
                L1.setPower(-0.2);
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
    public void move2(double SpeedA,double SpeedB,double SpeedC,double SpeedD, double kp_Turn, double kd_Turn, double en, double Turn){
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
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lastTime = 0;
        previous_odoturn = 0;
        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();

//        TelemetryPacket packet = new TelemetryPacket();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//
//        packet.fieldOverlay();
//        packet.fieldOverlay()
//                .setTranslation(0,144)
//                .setRotation(Math.PI / 2)
//                .setScale(1.0,1.0)
//                .drawImage("/images/intothedeep.webp", 0, 0, 144, 144)
//                .drawImage("/images/0.png", 0, 0, 1, 1);
//        dashboard.sendTelemetryPacket(packet);
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
            Delta_Turn_odo = Turn_odo - last_turn;
            forw = ((-MotorB.getCurrentPosition() - B_odo) + (MotorC.getCurrentPosition() - C_odo)) / 2.0;
            str = ((MotorD.getCurrentPosition() - D_odo) - (con * (Delta_Turn_odo)));

            x_odo += (forw * Math.cos(Turn_odo / 180.0 * Math.PI)) + ((str * Math.sin(Turn_odo / 180.0 * Math.PI)));
            y_odo += ((str * Math.cos(Turn_odo / 180.0 * Math.PI))) - (forw * Math.sin(Turn_odo / 180.0 * Math.PI));

            B_odo = -MotorB.getCurrentPosition();
            C_odo = MotorC.getCurrentPosition();
            D_odo = MotorD.getCurrentPosition();

            error_turn = Turn_odo - Turn;
            errorRate = (error_turn - previous_odoturn) ;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn);
            if (Math.abs(MotorA.getCurrentPosition()) < en){
                MotorA.setPower(SpeedA);
                MotorB.setPower(SpeedB);
                MotorC.setPower(SpeedC);
                MotorD.setPower(SpeedD);
            }
            else if (Math.abs(MotorA.getCurrentPosition()) > en){
                MotorA.setPower(SpeedA * -0.3);
                MotorB.setPower(SpeedB * -0.3);
                MotorC.setPower(SpeedC * -0.3);
                MotorD.setPower(SpeedD * -0.3);
            }
            else if (Math.abs(error_turn) >= 3){
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
        MotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        lastTime = 0;
        previous_odoturn = 0;
        myElapsedTime = new ElapsedTime();
        myElapsedTime.reset();

//        TelemetryPacket packet = new TelemetryPacket();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        telemetry = dashboard.getTelemetry();
//
//        packet.fieldOverlay();
//        packet.fieldOverlay()
//                .setTranslation(0,144)
//                .setRotation(Math.PI / 2)
//                .setScale(1.0,1.0)
//                .drawImage("/images/intothedeep.webp", 0, 0, 144, 144)
//                .drawImage("/images/0.png", 0, 0, 1, 1);
//        dashboard.sendTelemetryPacket(packet);
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
            Delta_Turn_odo = Turn_odo - last_turn;
            forw = ((-MotorB.getCurrentPosition() - B_odo) + (MotorC.getCurrentPosition() - C_odo)) / 2.0;
            str = ((MotorD.getCurrentPosition() - D_odo) - (con * (Delta_Turn_odo)));

            x_odo += (forw * Math.cos(Turn_odo / 180.0 * Math.PI)) + ((str * Math.sin(Turn_odo / 180.0 * Math.PI)));
            y_odo += ((str * Math.cos(Turn_odo / 180.0 * Math.PI))) - (forw * Math.sin(Turn_odo / 180.0 * Math.PI));

            B_odo = -MotorB.getCurrentPosition();
            C_odo = MotorC.getCurrentPosition();
            D_odo = MotorD.getCurrentPosition();

            error_turn = Turn_odo - Turn;
            errorRate = (error_turn - previous_odoturn) ;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn);
            if (Ch.getDistance(DistanceUnit.CM) >= dis){
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



