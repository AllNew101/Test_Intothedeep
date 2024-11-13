package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Config
@Autonomous(name = "backup2")
public class backup2 extends LinearOpMode {
    public static double con = 19;
    public static double kd = 0.001;
    public static double kp = -0.25;
    public static DcMotor MotorA;
    public static  DcMotor MotorB;
    public static DcMotor MotorC;
    public static DcMotor MotorD;
    private Servo RightServo;
    private Servo leftservo;

    public DcMotor L1;
    public Servo Wrist;

    public DcMotor L2;
    public Servo neep;
    public static double Turn_odo;
    public double Delta_Turn_odo;
    public static double D_odo;
    public static double x_odo;
    public static double y_odo;
    public static double B_odo;
    public static double C_odo;
    public double last_turn = 0;
    public double Rotate_with_Servo;


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
            neep.setPosition(0.1);
            move2(0.6,kd,kp,0,7150,0);
            up_sp(890);
            down_sp(890,350);

            neep.setPosition(0.3);
            L1.setPower(-0.6);
            L2.setPower(-0.6);
            sleep(300);
            L1.setPower(0);
            L2.setPower(0);
            neep.setPosition(0.1);
            servo_wolfpack(0.7);
            move2(0.6,kd,kp,0,4450,0);
            move2(0.6,kd,kp,10700,4450,0);
            neep.setPosition(0.4);
            servo_wolfpack(0.1);
            Wrist.setPosition(0.93);
            sleep(500);
            neep.setPosition(0.1);

            sleep(1000);



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
        while (true){
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

            servo_wolfpack(0.52);
            Wrist.setPosition(0.89);

        }
        }}
    public void down_sp(int UP,int Down) {
        while (true){
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
        public void move2(double Speed, double kp_Turn, double kd_Turn, int y_positions, int x_positions, int Turn){
            double movement;
            double error_sum;
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

            MotorA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorC.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            MotorD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            y_positions = y_positions * -1;
            lastTime = 0;
            previous_odoturn = 0;
            myElapsedTime = new ElapsedTime();
            error_sum = 0;
            error_sum_XY = 0;
            errorRate_XY = 0;
            myElapsedTime.reset();
            slowY = (y_positions - y_odo) * 0.8;
            slowX = (x_positions - x_odo) * 0.8;
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
                telemetry.update();


                Turn_odo = ((-MotorB.getCurrentPosition()) - (MotorC.getCurrentPosition())) / 72;
                Delta_Turn_odo = Turn_odo - (int) last_turn;//((-MotorB.getCurrentPosition() - B_odo) - (-MotorC.getCurrentPosition() - C_odo)) / 54;
                forw = ((-MotorB.getCurrentPosition() - B_odo) + (MotorC.getCurrentPosition() - C_odo)) / 2.0;
                str = ((MotorD.getCurrentPosition() - D_odo) - (con * (Delta_Turn_odo)));

                x_odo += (forw * Math.cos(Turn_odo / 180.0 * Math.PI)) + ((str * Math.sin(Turn_odo / 180.0 * Math.PI)));
                y_odo += ((str * Math.cos(Turn_odo / 180.0 * Math.PI))) - (forw * Math.sin(Turn_odo / 180.0 * Math.PI));

                B_odo = -MotorB.getCurrentPosition();
                C_odo = MotorC.getCurrentPosition();
                D_odo = MotorD.getCurrentPosition();

                error_turn = Turn - Turn_odo;
                errorRate = (error_turn - previous_odoturn) ;
                dt = myElapsedTime.milliseconds() - lastTime;
//            error_sum += error_turn * dt;
                movement = (error_turn * kp_Turn) + (errorRate * kd_Turn);
                degree = Math.atan2(x_positions - x_odo, y_positions - y_odo) - (Turn_odo / 180.0 * Math.PI);
//            x_degree = ((x_positions - x_odo) * Math.cos(Turn_odo)) - ((y_positions - y_odo) * Math.sin(Turn_odo));
//            y_degree = ((y_positions - y_odo) * Math.cos(Turn_odo) + (x_positions - x_odo) * Math.sin(Turn_odo));

                //y_positions - y_odo >= 100 || y_positions - y_odo <= -100 || x_positions - x_odo >= 100 || x_positions - x_odo <= -100
//          degree = (Math.atan2(x_degree,y_degree) - Math.PI/4.0);
                //Math.abs(Math.sqrt(Math.pow(y_positions - y_odo,2) + Math.pow(x_positions - x_odo,2))) >= 200
                degree = (degree - Math.PI / 4.0);

                if (y_positions - y_odo >= 1000 || y_positions - y_odo <= -1000 || x_positions - x_odo >= 1000 || x_positions - x_odo <= -1000 || Math.abs(Turn - Turn_odo) >= 4) {
                    telemetry.addData("Check", "out");
                    MotorA.setPower(Math.min(Math.max((Math.sin(degree) * Speed + (movement)), -Speed), Speed));
                    MotorB.setPower(Math.min(Math.max((Math.cos(degree) * Speed - (movement)), -Speed), Speed));
                    MotorC.setPower(Math.min(Math.max((Math.cos(degree) * Speed + (movement)), -Speed), Speed));
                    MotorD.setPower(Math.min(Math.max((Math.sin(degree) * Speed - (movement)), -Speed), Speed));
                } else if (y_positions - y_odo >= 300 || y_positions - y_odo <= -300 || x_positions - x_odo >= 300 || x_positions - x_odo <= -300 || Math.abs(Turn - Turn_odo) >= 3) {
                    telemetry.addData("Check", "in");
                    MotorA.setPower(Math.min(Math.max((Math.sin(degree) * 0.15 + (movement)), -0.15), 0.15));
                    MotorB.setPower(Math.min(Math.max((Math.cos(degree) * 0.15- (movement)), -0.15), 0.15));
                    MotorC.setPower(Math.min(Math.max((Math.cos(degree) * 0.15 + (movement)), -0.15), 0.15));
                    MotorD.setPower(Math.min(Math.max((Math.sin(degree) * 0.15 - (movement)), -0.15), 0.15));
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
                last_turn = Turn_odo;
            }
        }


    }



