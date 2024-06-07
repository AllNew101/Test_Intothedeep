package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "backup2")
public class backup2 extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor MotorA;
    private DcMotor MotorB;
    private DcMotor MotorC;
    private DcMotor MotorD;
    private DcMotor MotorF;

    double Turn_odo;
    double Delta_Turn_odo;
    double D_odo;
    double x_odo;
    double y_odo;
    double B_odo;
    double C_odo;
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
        MotorF = hardwareMap.get(DcMotor.class, "MotorF");
        initAprilTag();
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
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


            MotorF.setPower(1);
            sleep(1000);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 0, 25000, 0, 1);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 5000, 25000, 0,0);
            move2(0.7, 0.01, 0,0.25,0.2, 0.01,0.3, 0, 25000, 0,0);
            move2(0.5, 0.01, 0, 0.25,0.2, 0.01,0.3, -600, 0, 0, 1);
            MotorF.setPower(1);
            sleep(1000);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 0, 25000, 0, 1);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 5000, 25000, 0,0);
            move2(0.7, 0.01, 0,0.25,0.2, 0.01,0.3, 0, 25000, 0,0);
            move2(0.5, 0.01, 0, 0.25,0.2, 0.01,0.3, -800, 0, 0, 1);
            MotorF.setPower(1);
            sleep(1000);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 0, 25000, 0, 1);
            move2(0.7, 0.01, 0, 0.25,0.2, 0.01,0.3, 5000, 25000, 0,0);

            visionPortal.close();






        }
    }

    /**
     * Describe this function...
     */
    private void move2(double Speed, double kp_Turn, double ki_Turn,  double kd_Turn, double kp_XY, double ki_XY,  double kd_XY, int y_positions, int x_positions, int Turn, int intake) {
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

        x_positions = x_positions * -1;
        lastTime = 0;
        previous_odoturn = 0;
        myElapsedTime = new ElapsedTime();
        error_sum = 0;
        error_sum_XY = 0;
        errorRate_XY = 0;
        myElapsedTime.reset();
        slowY = y_positions / 10;
        slowX = x_positions / 10;
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
            error_sum += error_turn * dt;
            movement = (error_turn * kp_Turn) + (errorRate * kd_Turn) + (error_sum * ki_Turn);
            degree = Math.atan2(x_positions - x_odo, y_positions - y_odo)  - (Turn_odo / 180.0 * Math.PI) ;
            x_degree = ((x_positions - x_odo) * Math.cos(Turn_odo)) - ((y_positions - y_odo) * Math.sin(Turn_odo));
            y_degree = ((y_positions - y_odo) * Math.cos(Turn_odo) + (x_positions - x_odo) * Math.sin(Turn_odo));

            //y_positions - y_odo >= 100 || y_positions - y_odo <= -100 || x_positions - x_odo >= 100 || x_positions - x_odo <= -100
//          degree = (Math.atan2(x_degree,y_degree) - Math.PI/4.0);
            //Math.abs(Math.sqrt(Math.pow(y_positions - y_odo,2) + Math.pow(x_positions - x_odo,2))) >= 200
            degree = (degree - Math.PI/4.0);
            if ((y_positions - y_odo >= 200 && Math.abs(y_positions - y_odo) <= slowY)|| (y_positions - y_odo <= -200 && Math.abs(y_positions - y_odo) <= slowY) || (x_positions - x_odo >= 200 && Math.abs(x_positions - x_odo) <= slowX) || (x_positions - x_odo <= -200 && Math.abs(x_positions - x_odo) <= slowX )|| Math.abs(Turn - Turn_odo) >= 3) {
                error_XY = Math.sqrt(Math.pow(y_positions, 2) + Math.pow(x_positions, 2)) - Math.sqrt(Math.pow(y_odo, 2) + Math.pow(x_odo, 2));
                errorRate_XY = ((error_XY - previous_odoXY) / dt);

                if (Math.abs(error_XY) <= 100) {
                    error_sum_XY += error_XY * dt;
                }
                pidXY = (error_XY * kp_XY) + (errorRate_XY * kd_XY) + (error_sum_XY * ki_XY);

                MotorA.setPower(Math.min(Math.max((Math.sin(degree) * 0.3 + (movement)) , -0.3), 0.3) );
                MotorB.setPower(Math.min(Math.max((Math.cos(degree) * 0.3 - (movement)) , -0.3), 0.3) );
                MotorC.setPower(Math.min(Math.max((Math.cos(degree) * 0.3 + (movement)) , -0.3), 0.3) );
                MotorD.setPower(Math.min(Math.max((Math.sin(degree) * 0.3 - (movement)) , -0.3), 0.3) );
                if(intake == 1 ) {
                    MotorF.setPower(1);
                }


                telemetry.addData("Turn done?",Math.abs(Turn - Turn_odo) < 3);
                telemetry.addData("Position done?",Math.abs(Math.sqrt(Math.pow(y_positions - y_odo,2) + Math.pow(x_positions - x_odo,2))) < 100);
                telemetry.update();
            }
            else if (y_positions - y_odo >= 200 || y_positions - y_odo <= -200 || x_positions - x_odo >= 200 || x_positions - x_odo <= -200 || Math.abs(Turn - Turn_odo) >= 3) {
                MotorA.setPower(Math.min(Math.max((Math.sin(degree) * Speed + (movement)) , -Speed), Speed) );
                MotorB.setPower(Math.min(Math.max((Math.cos(degree) * Speed - (movement)) , -Speed), Speed) );
                MotorC.setPower(Math.min(Math.max((Math.cos(degree) * Speed + (movement)) , -Speed), Speed) );
                MotorD.setPower(Math.min(Math.max((Math.sin(degree) * Speed - (movement)) , -Speed), Speed) );
                if(intake == 1 ) {
                    MotorF.setPower(1);
                }
            }
            else {
                MotorA.setPower(0);
                MotorB.setPower(0);
                MotorC.setPower(0);
                MotorD.setPower(0);
                MotorF.setPower(0);
                MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                MotorF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
    private void initAprilTag() {

        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


    }   // end method telemetryAprilTag()

    private void april(){
        boolean a = true;
        while (opModeIsActive()) {
            while(a == true) {


                //telemetryAprilTag();
                // Push telemetry to the Driver Station.
                telemetry.update();


                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                telemetry.addData("# AprilTags Detected", currentDetections.size());
                for (AprilTagDetection detection : currentDetections) {

                    // Step through the list of detections and display info for each one.
                    if (detection.metadata != null) {
                        if(detection.id == 4){
                            MotorA.setPower(-0.15);
                            MotorB.setPower(0.15);
                            MotorC.setPower(0.15);
                            MotorD.setPower(-0.15);
                            sleep(300);
                        }
                        else if(detection.id == 5){
                            MotorA.setPower(0);
                            MotorB.setPower(0);
                            MotorC.setPower(0);
                            MotorD.setPower(0);
                            MotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            MotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                            a = false;

                        }
                        else {
                            MotorA.setPower(-0.15);
                            MotorB.setPower(0.15);
                            MotorC.setPower(0.15);
                            MotorD.setPower(-0.15);
                            sleep(300);


                        }

                    } else {
                        MotorA.setPower(-0.2);
                        MotorB.setPower(0.2);
                        MotorC.setPower(0.2);
                        MotorD.setPower(-0.2);
                        sleep(500);

                    }
                    if (detection.metadata != null) {
                        telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                        telemetry.update();
                    } else {
                        telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                        telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                    }


                }

                // Save CPU resources; can resume streaming when needed.

            }
            // Share the CPU.

        }
    }
}


