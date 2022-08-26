package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;


@TeleOp(name = "Chandrayan TeleOp 2", group = "Yesir")
public class CTeleOp2 extends OpMode {

    private DcMotor rightFront;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor tape;
    Servo leftFoundation;
    Servo rightFoundation;
    Servo capstoneGrab;
    private DcMotor up;
    private DcMotor linear;
    private Servo grabber;
    double grabPos = 0.2;
    double releasePos = 0.75;
    double halfReleasePos = 0.45;
    int upPosition = 0;
    boolean liftFirstTime = true;
    DistanceSensor rightDist;
    DistanceSensor leftDist;
    int[] postions = {0, 380, 670, 920, 1214, 1500, 1740, 2000, 2230, 2530, 2750, 3030};
    double linearTime = 0;
    double turnDivide = 1.7;

    double blockTime = 0;

    boolean openGrabFull = false;
    double openGrabFullTime = 0;


    @Override
    public void init() {
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        tape = hardwareMap.get(DcMotor.class, "tape");
        up = hardwareMap.get(DcMotor.class, "up");
        linear = hardwareMap.get(DcMotor.class, "linear");
        up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber = hardwareMap.get(Servo.class, "grabber");
        leftFoundation = hardwareMap.get(Servo.class, "leftFoundation");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        capstoneGrab = hardwareMap.get(Servo.class, "capstone");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        //linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        up.setTargetPosition(0);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setPower(1);
        rightDist = hardwareMap.get(DistanceSensor.class, "rightColor");
        leftDist = hardwareMap.get(DistanceSensor.class, "leftColor");
        tape.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {

        super.start();
        linear.setPower(0);
        linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capstoneGrab.setPosition(0.68);
//        leftFoundation.setPosition(0.7);
//        rightFoundation.setPosition(0.2);
        grabber.setPosition(releasePos);
        foundationUngrab();


    }

    @Override
    public void loop() {

        double power;

//        telemetry.addData("Linear", linear.getCurrentPosition());
//        telemetry.update();

//        if (up.getCurrentPosition() > 500 && getRuntime() < 100) {
//            power = 0.2;
//        } else {
        if (gamepad1.left_trigger > 0.5) {
            turnDivide = 2;
            if (Math.abs(gamepad1.left_stick_x) > 0.6 || Math.abs(gamepad1.right_stick_x)> 0.4) {
                power = 0.7;
            }
            else if(gamepad1.left_stick_y>0.2){
                power = 0.4;
            }
            else {
                power = 0.25;
            }
        } else {
            power = 1.5;
            turnDivide = 1.7;
        }
        //}




        double linearPower = Math.abs(gamepad2.left_trigger - 1) / 1.7 + 0.33;
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y) * 1.6;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX / turnDivide;
        final double v2 = r * Math.sin(robotAngle) + rightX / turnDivide;
        final double v3 = r * Math.sin(robotAngle) - rightX / turnDivide;
        final double v4 = r * Math.cos(robotAngle) + rightX / turnDivide;

        leftFront.setPower(power * v1);
        rightFront.setPower(power * v2);
        leftBack.setPower(power * v3);
        rightBack.setPower(power * v4);

//        double rightVal = (rightColor.red() * rightColor.green())/Math.pow(rightColor.blue(), 2);
//        double leftVal = (leftColor.red() * leftColor.green())/Math.pow(leftColor.blue(), 2);
//
//        telemetry.addData("left", leftVal);
//        telemetry.addData("right", rightVal);
//        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.CM));
//        telemetry.update();

//        telemetry.addData("LEFT", "");
//        telemetry.addData("red", leftColor.red());
//        telemetry.addData("green", leftColor.green());
//        telemetry.addData("blue", leftColor.blue());
//        telemetry.addData("alpha", leftColor.alpha());
//        telemetry.addData("argb", leftColor.argb());
//
//        telemetry.addData("RIGHT", "");
//        telemetry.addData("red", rightColor.red());
//        telemetry.addData("green", rightColor.green());
//        telemetry.addData("blue", rightColor.blue());
//        telemetry.addData("alpha", rightColor.alpha());
//        telemetry.addData("argb", rightColor.argb());

        //telemetry.update();

        tape.setPower(gamepad2.right_stick_y);

        if (gamepad2.a) {
            if (gamepad2.left_trigger < 0.5) {
                upPosition = postions[0];
            } else {
                upPosition = postions[8];
            }
        } else if (gamepad2.b) {
            if (gamepad2.left_trigger < 0.5) {
                upPosition = postions[1];
            } else {
                upPosition = postions[9];
            }
        } else if (gamepad2.y) {
            if (gamepad2.left_trigger < 0.5) {
                upPosition = postions[2];
            } else {
                upPosition = postions[10];
            }
        } else if (gamepad2.x) {
            if (gamepad2.left_trigger < 0.5) {
                upPosition = postions[3];
            } else {
                upPosition = postions[11];
            }
        } else if (gamepad2.dpad_down) {
            upPosition = postions[4];
        } else if (gamepad2.dpad_right) {
            upPosition = postions[5];
        } else if (gamepad2.dpad_up) {
            upPosition = postions[6];
        } else if (gamepad2.dpad_left) {
            upPosition = postions[7];
        } else if (gamepad2.left_bumper || gamepad2.right_bumper) {
            if (liftFirstTime) {
                if (gamepad2.left_bumper) {
                    upPosition -= 100;
                }
                if (gamepad2.right_bumper) {
                    upPosition += 100;
                }
                liftFirstTime = false;
            }
        } else {
            liftFirstTime = true;
        }

        //clamp this later
        upPosition = clamp(upPosition, 0, 3300);

        up.setTargetPosition(upPosition);

        if(upPosition == 0){
            up.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            up.setPower(0);
        }
        else{
            up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            up.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            up.setPower(1);
        }

        //double a = gamepad2.right_trigger;

        if (gamepad1.a) {
            grabber.setPosition(grabPos);
        } else if (gamepad1.x) {
            grabber.setPosition(halfReleasePos);
        }
        else if(gamepad1.b){
            grabber.setPosition(releasePos);
        }
        //Right trigger to open grabber and center - tell Suhas


        if (gamepad1.right_bumper) { // Grab
            foundationGrab();
        } else if (gamepad1.left_bumper){  // Ungrab
            foundationUngrab();
        }

        if(gamepad1.dpad_down){
            upPosition = 1;
        }



        if(gamepad2.back){
            blockTime = getRuntime() + 1;
        }
        else if(gamepad2.right_trigger > 0.4){
            linearTime = getRuntime() + 1;
            openGrabFullTime = getRuntime() + 2;
            openGrabFull = true;
        }
        if(blockTime>getRuntime() && linear.getCurrentPosition() > -1700 && linear.getCurrentPosition() < 1500){
            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(leftDist.getDistance(DistanceUnit.CM)<5){
                linear.setPower(0.7);
            }
            else if(rightDist.getDistance(DistanceUnit.CM)<5){
                linear.setPower(-0.7);
            }
            else
            {
                linear.setPower(0);
            }
        }
        else if(linearTime>getRuntime()){
            linear.setTargetPosition(0);
            linear.setPower(1);
            linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            grabber.setPosition(releasePos);
        }

        else{
            linear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            linear.setPower(gamepad2.left_stick_x * linearPower);
        }

        if(openGrabFullTime<getRuntime()&&openGrabFull){
            openGrabFull = false;
            grabber.setPosition(halfReleasePos);
        }


        //Capstone stuff for Akshat

        if(gamepad1.dpad_right){
            //This is release
            capstoneGrab.setPosition(0.5);
        }
        else if (gamepad1.dpad_left){
            //This is grab
            capstoneGrab.setPosition(0.69);
        }

//        if(gamepad2.back){
//            gay = true;
//        }

//        telemetry.addData("up", up.getCurrentPosition());
//        telemetry.update();

        telemetry.addData("left", leftDist.getDistance(DistanceUnit.CM));
        telemetry.addData("right", rightDist.getDistance(DistanceUnit.CM));
//////        telemetry.addData("blockTime", blockTime);
//////        telemetry.addData("time", getRuntime());
        telemetry.update();

    }

    public static int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void stop() {
        super.stop();
    }

    private void foundationGrab(){
        leftFoundation.setPosition(0.46);
        rightFoundation.setPosition(0.75);
    }

    private void foundationUngrab(){
        leftFoundation.setPosition(0.78);
        rightFoundation.setPosition(0.41);
    }
}



