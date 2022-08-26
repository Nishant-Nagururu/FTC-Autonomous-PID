// Simple autonomous program that drives bot forward until end of period
// or touch sensor is hit. If touched, backs up a bit and turns 90 degrees
// right and keeps going. Demonstrates obstacle avoidance and use of the
// REV Hub's built in IMU in place of a gyro. Also uses gamepad1 buttons to
// simulate touch sensor press and supports left as well as right turn.
//
// Also uses PID controller to drive in a straight line when not
// avoiding an obstacle.
//
// Use PID controller to manage motor power during 90 degree turn to reduce
// overshoot.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Chandrayan Red", group="Autonomous")
public class CRed extends LinearOpMode
{
    DcMotor leftFront, rightFront, leftBack, rightBack, up, linear;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .7, correction, rotation;
    PIDController pidRotate, pidDrive, pidStrafe;
    Servo grabber;
    Servo leftFoundation;
    Servo rightFoundation;
    Servo capstone;
    double grabPos = 0.2;
    double releasePos = 0.75;
    double halfReleasePos = 0.45;
    double startPos = 0.8;
    ColorSensor rightColor;
    ColorSensor leftColor;
    int upPosition = 500;
    DistanceSensor leftDist;

    DistanceSensor rightDist;
    double capGrab = 0.69;

    int upPositionStart = 130;

    double rightLeftPower = 0.7;

    int distToRightOne = 50;
    int distToRightTwo = distToRightOne - 20;
    int distMiddle = 10;
    int distRight = 33;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initialization();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        waitForStart();

        starting();

        //sleep(2000);

        sensing();
//
        finish();
    }

    private void sensing() {
        drive(1.2, 0.5);


        double rightVal = (rightColor.red() * rightColor.green())/Math.pow(rightColor.blue(), 2);
        double leftVal = (leftColor.red() * leftColor.green())/Math.pow(leftColor.blue(), 2);

        telemetry.addData("left", leftVal);
        telemetry.addData("right", rightVal);
        telemetry.update();

        if (rightVal > 3.2 && leftVal > 3.2){
            middleBlock();
        }
        else if(rightVal > leftVal){
            leftBlock();
        }
        else{
            rightBlock();
        }
    }

    private void middleBlock(){

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.3,0.5);

        side(3.15, distToRightOne);

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        up.setTargetPosition(300);

        driveRelease(0.45,0.5, false);

        driveBack(0.4, 0.5);

        up.setPower(0.3);
        up.setTargetPosition(0);

        side(-4.2, distMiddle);//power 0.7

        telemetry.addData("leftDist", leftDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        drive(0.36, 0.5);

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.5, 0.5);

        side(4.6, distToRightTwo);//power 0.7

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    double leftMove = 0.3;

    private void leftBlock(){

        side(-leftMove, 0);

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.3,0.5);

        side(3.15 + leftMove + 0.1, distToRightOne);

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        up.setTargetPosition(300);

        driveRelease(0.45,0.5, false);

        driveBack(0.4, 0.5);

        up.setPower(0.3);
        up.setTargetPosition(0);

        linear.setTargetPosition(-1500);
        linear.setPower(1);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        side(-4.1 - leftMove - 0.1, 0);//power 0.7

        telemetry.addData("leftDist", leftDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        drive(0.4, 0.5);

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.55, 0.5);

        linear.setTargetPosition(0);

        side(4.6 + leftMove - 0.1, distToRightTwo);//power 0.7

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    double rightMove = 0.3;

    private void rightBlock(){

        side(rightMove, 0);

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.3,0.5);

        side(3.15 - rightMove, distToRightOne);

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        up.setTargetPosition(300);

        driveRelease(0.45,0.5, false);

        driveBack(0.4, 0.5);

        up.setPower(0.3);
        up.setTargetPosition(0);

        side(-4.2 + leftMove, distRight);//power 0.7

        telemetry.addData("leftDist", leftDist.getDistance(DistanceUnit.CM));
        telemetry.update();

        drive(0.4, 0.5);

        driveGrab();

        up.setTargetPosition(upPositionStart);

        driveBack(0.55, 0.5);

        side(4.6 - leftMove, distToRightTwo);//power 0.7

        telemetry.addData("rightDist", rightDist.getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    private void finish(){

        up.setTargetPosition(upPositionStart + 200);

        foundationHalfGrab();

        driveRelease(0.6, 0.4, true);

        sleep(300);

//        diagLeft(0.5, -1);
//        driveBack(0.3, 1);
//
//        rotate(-90, 0.6);
//
//        setPowers(0.6);
//        sleep(500);
//        setPowers(0);
//
//        foundationUnGrab();
//
//        //up.setTargetPosition(100);
//
//        driveRightNormal(0.35, -0.7);
//
//        lastBack(1.7, -1);

        diagRight(0.2, -1);
        driveNormal(1.8, -1);
        foundationUnGrab();
        sleep(200);
        driveRightNormal(1.4, -1);
        up.setTargetPosition(0);
        driveNormal(1, 1);
        grabber.setPosition(releasePos);
        //side(-1, 0);
        driveRightNormal(1, -1);
    }


    private void driveBack(double distance, double power){
        drive(distance, -power);
    }

    private void driveLeft(double distance, double power, double distWall){
        driveRight(distance, -power, -distWall);
    }

    private void grab() {
        grabber.setPosition(grabPos);
        sleep(500);
    }

    private void foundationGrab() {
        leftFoundation.setPosition(0.45);
        rightFoundation.setPosition(0.75);
    }
    private void foundationUnGrab() {
        leftFoundation.setPosition(0.75);
        rightFoundation.setPosition(0.45);
    }

    private void foundationHalfGrab(){
        leftFoundation.setPosition(0.6);
        rightFoundation.setPosition(0.6);
    }

    private void release() {
        grabber.setPosition(releasePos);
        //sleep(200);
    }

    private void driveRelease(double distance, double power, boolean grab){
        int dist = (int)(distance * 950);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        up.setPower(1);

        double addVal = 0.3;
        if (power < 0){
            addVal = -0.3;
        }
        double divideVal = 600/(Math.abs(power) - 0.3);
        if(power < 0){
            divideVal = -divideVal;
        }

        while(opModeIsActive() && Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2)<dist){
            double distanceDriven = Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2);
            double distanceRemaining = dist - distanceDriven;
            if(distanceRemaining<=600) {
                up.setTargetPosition(upPosition);
                if(distanceRemaining <= 150){
                    grabber.setPosition(releasePos);
                    if(distanceRemaining<= 200 && grab){
                        foundationGrab();
                    }
                }
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(distanceRemaining/divideVal + addVal - correction/2);
                rightFront.setPower(distanceRemaining/divideVal + addVal + correction/2);
                leftBack.setPower(distanceRemaining/divideVal + addVal - correction/2);
                rightBack.setPower(distanceRemaining/divideVal + addVal + correction/2);
                idle();
            }
            else if(distanceDriven <= 200){
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(distanceDriven/divideVal + addVal - correction/2);
                rightFront.setPower(distanceDriven/divideVal + addVal + correction/2);
                leftBack.setPower(distanceDriven/divideVal + addVal - correction/2);
                rightBack.setPower(distanceDriven/divideVal + addVal + correction/2);
                idle();
                if(distanceDriven>150&& grab){
                    foundationHalfGrab();
                }
            }
            else{
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(power - correction);
                rightFront.setPower(power + correction);
                leftBack.setPower(power - correction);
                rightBack.setPower(power + correction);
                idle();
            }
        }
        stopBot();
        sleep(100);
    }

    private void driveGrab() {
        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);
        int dist = 100;

        while(opModeIsActive() && Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2)<dist) {
            int distanceDriven = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2;
            if(distanceDriven>30){
                grabber.setPosition(grabPos);
            }

            leftFront.setPower(0.2);
            rightFront.setPower(0.2);
            leftBack.setPower(0.2);
            rightBack.setPower(0.2);

        }
        stopBot();
        sleep(100);
    }

    private void driveNormal(double distance, double power){
        int dist = (int)(distance * 964);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2)<dist) {
            int distanceDriven = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2;
            int distRemaining = dist - distanceDriven;
            if(distRemaining<= 1000){
                leftFront.setPower(power/2);
                rightFront.setPower(power/2);
                leftBack.setPower(power/2);
                rightBack.setPower(power/2);
            }
            else{
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
            }

        }
        stopBot();
        sleep(100);

    }

    private void driveRightNormal(double distance, double power){
        int dist = (int)(distance * 1050);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2 < dist){
            int distanceDriven = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2;
            int distRemaining = dist - distanceDriven;
            if (distRemaining <= 500){
                leftFront.setPower(power/2);
                rightFront.setPower(-power/2);
                leftBack.setPower(-power/2);
                rightBack.setPower(power/2);
            }
            else{
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
            }
        }
        stopBot();
        sleep(100);
    }

    private void drive(double distance, double power) {

        int dist = (int)(distance * 950);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        double addVal = 0.3;
        if (power < 0){
            addVal = -0.3;
        }
        double divideVal = 1000/(Math.abs(power) - 0.3);
        if(power < 0){
            divideVal = -divideVal;
        }

        while(opModeIsActive() && Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2)<dist){
            double distanceDriven = Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2);
            double distanceRemaining = dist - distanceDriven;
            if(distanceRemaining<=1000) {
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(distanceRemaining/divideVal + addVal - correction/2);
                rightFront.setPower(distanceRemaining/divideVal + addVal + correction/2);
                leftBack.setPower(distanceRemaining/divideVal + addVal - correction/2);
                rightBack.setPower(distanceRemaining/divideVal + addVal + correction/2);
                idle();
            }
            else if(distanceDriven <= 200){
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(distanceDriven/divideVal + addVal - correction/2);
                rightFront.setPower(distanceDriven/divideVal + addVal + correction/2);
                leftBack.setPower(distanceDriven/divideVal + addVal - correction/2);
                rightBack.setPower(distanceDriven/divideVal + addVal + correction/2);
                idle();
            }
            else{
                correction = pidDrive.performPID(getAngle());
                leftFront.setPower(power - correction);
                rightFront.setPower(power + correction);
                leftBack.setPower(power - correction);
                rightBack.setPower(power + correction);
                idle();
            }
        }
        stopBot();
        sleep(100);
    }

    private void lastBack(double distance, double power){
        int dist = (int)(distance * 950);

        while(opModeIsActive() && Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2)<dist){
            double distanceDriven = Math.abs((leftFront.getCurrentPosition() + rightFront.getCurrentPosition())/2);
            if(distanceDriven > 700){
                up.setTargetPosition(0);
            }
            correction = pidDrive.performPID(getAngle());
            leftFront.setPower(power - correction);
            rightFront.setPower(power + correction);
            leftBack.setPower(power - correction);
            rightBack.setPower(power + correction);
            idle();
        }

        stopBot();
        sleep(200);
    }


    private void side(double distance, int distToWall) {

        double thisPower = 0.6;
        boolean left = false;

        int dist = Math.abs((int)(distance * 1100));

        if (distance<0){
            left = true;
        }

        double leftMotorPower = 0.65 * 1.2;
        double rightBackPower = 0.61* 1.2;
        double rightMotorPower = -0.73* 1.2;
        double leftBackPower = -0.6* 1.2;

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if(left){
            leftMotorPower = -0.7 * 1.2;
            rightBackPower = -0.64* 1.2;
            rightMotorPower = 0.7* 1.2;
            leftBackPower = 0.7* 1.2;
            distToWall = -distToWall;
        }

        double addValue = 0.2;
        if (left){
            addValue = -0.2;
        }
        double divideVal = 500/(Math.abs(thisPower) - 0.2);
        if(left){
            divideVal = -divideVal;
        }

        while(opModeIsActive() && (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition()))/2 < dist){
            int distanceDriven = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition()))/2;
            int distRemaining = dist - distanceDriven;
            if(distRemaining <= 500) {
                correction = pidStrafe.performPID(getAngle());
                leftFront.setPower(distRemaining/divideVal + addValue - correction);
                rightBack.setPower((distRemaining/divideVal + addValue) * 0.875 + correction);
                rightFront.setPower((-distRemaining/divideVal - addValue) * 1.08 + correction);
                leftBack.setPower((-distRemaining/divideVal - addValue) * 1.15  - correction);
                idle();
            }
            else if(distanceDriven <= 400){
                correction = pidStrafe.performPID(getAngle());
                leftFront.setPower(distanceDriven/divideVal + addValue - correction);
                rightBack.setPower((distanceDriven/divideVal + addValue) * 0.875 + correction);
                rightFront.setPower((-distanceDriven/divideVal - addValue) * 1.08 + correction);
                leftBack.setPower((-distanceDriven/divideVal - addValue) * 1.2 - correction);
                idle();
            }
            else{
                correction = pidStrafe.performPID(getAngle());
                leftFront.setPower(leftMotorPower - (correction));
                rightBack.setPower(rightBackPower + (correction));
                rightFront.setPower(rightMotorPower + (correction));
                leftBack.setPower(leftBackPower - (correction));
                telemetry.addData("correctiona", correction);
                telemetry.update();
                idle();
            }
        }

        stopBot();

        sleep(100);

        if(distToWall < 0){
            while(leftDist.getDistance(DistanceUnit.CM) > Math.abs(distToWall) + 2){
                leftFront.setPower(-0.3);
                rightBack.setPower(-0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(0.3);
            }
            while(leftDist.getDistance(DistanceUnit.CM) < Math.abs(distToWall) - 2){
                leftFront.setPower(0.3);
                rightBack.setPower(0.3);
                rightFront.setPower(-0.3);
                leftBack.setPower(-0.3);
            }
        }
        else if(distToWall > 0){
            while(rightDist.getDistance(DistanceUnit.CM) > Math.abs(distToWall) + 2){
                leftFront.setPower(0.3);
                rightBack.setPower(0.3);
                rightFront.setPower(-0.3);
                leftBack.setPower(-0.3);
            }
            while(rightDist.getDistance(DistanceUnit.CM) < Math.abs(distToWall) - 2){
                leftFront.setPower(-0.3);
                rightBack.setPower(-0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(0.3);
            }
        }

        stopBot();
        sleep(100);
    }
    //nishant helped drastically. srikar did absolutly nothing he did jackshit he is a useless piece of garbage
    private void driveRight(double distance, double power, double distWall) {
        int dist = (int)(distance * 1100);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        double addValue = 0.2;
        if (power < 0){
            addValue = -0.2;
        }
        double divideVal = 1000/(Math.abs(power) - 0.2);
        if(power < 0){
            divideVal = -divideVal;
        }

        while(opModeIsActive() && (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2 < dist){
            int distanceDriven = (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2;
            int distRemaining = dist - distanceDriven;
//            if(distRemaining <= 1000) {
//                correction = pidStrafe.performPID(getAngle());
//                leftMotor.setPower(distRemaining/divideVal + addValue - correction);
//                rightBack.setPower(distRemaining/divideVal + addValue + correction- 0.07);
//                rightMotor.setPower(-distRemaining/divideVal - addValue + correction);
//                leftBack.setPower(-distRemaining/divideVal - addValue - correction+ 0.07);
//                idle();
//            }
//            else if(distanceDriven <= 400){
//                correction = pidStrafe.performPID(getAngle());
//                leftMotor.setPower(distanceDriven/divideVal + addValue - correction);
//                rightBack.setPower(distanceDriven/divideVal + addValue + correction- 0.07);
//                rightMotor.setPower(-distanceDriven/divideVal - addValue + correction);
//                leftBack.setPower(-distanceDriven/divideVal - addValue - correction+ 0.07);
//                idle();
//            }
//            else{
                correction = pidStrafe.performPID(getAngle());
                leftFront.setPower(power - (correction * 1.3));
                rightBack.setPower(power + (correction * 1.3));
                rightFront.setPower(-power + (correction * 1.3));
                leftBack.setPower(-power - (correction * 1.3));
                idle();

                telemetry.addData("correction", correction);
                telemetry.update();
            //}
        }

        stopBot();

        //sleep(200);
        sleep(200);

        if(distWall < 0){
            while(leftDist.getDistance(DistanceUnit.CM) > Math.abs(distWall) + 2){
                leftFront.setPower(-0.3);
                rightBack.setPower(-0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(0.3);
            }
            while(leftDist.getDistance(DistanceUnit.CM) < Math.abs(distWall) - 2){
                leftFront.setPower(0.3);
                rightBack.setPower(0.3);
                rightFront.setPower(-0.3);
                leftBack.setPower(-0.3);
            }
        }
        else if(distWall > 0){
            while(rightDist.getDistance(DistanceUnit.CM) > Math.abs(distWall) + 2){
                leftFront.setPower(0.3);
                rightBack.setPower(0.3);
                rightFront.setPower(-0.3);
                leftBack.setPower(-0.3);
            }
            while(rightDist.getDistance(DistanceUnit.CM) < Math.abs(distWall) - 2){
                leftFront.setPower(-0.3);
                rightBack.setPower(-0.3);
                rightFront.setPower(0.3);
                leftBack.setPower(0.3);
            }
        }

        stopBot();

        //sleep(200);
        sleep(200);
    }

    private void diagLeft(double distance, double power) {
        int dist = (int)(distance * 1350);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (Math.abs(leftFront.getCurrentPosition()) + Math.abs(rightBack.getCurrentPosition()))/2 < dist){
            correction = pidDrive.performPID(getAngle());
            leftFront.setPower(power - correction);
            rightBack.setPower(power + correction);
            leftBack.setPower(-correction);
            rightFront.setPower(correction);
            idle();
        }

        stopBot();

        sleep(100);
    }

    private void diagRight(double distance, double power) {
        int dist = (int)(distance * 1350);

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while(opModeIsActive() && (Math.abs(leftBack.getCurrentPosition()) + Math.abs(rightFront.getCurrentPosition()))/2 < dist){
            correction = pidDrive.performPID(getAngle());
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(correction);
            leftFront.setPower(-correction);
            idle();
        }

        stopBot();

        sleep(100);
    }

    private void stopBot() {
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        leftBack.setPower(0);
    }

    public void setModes(DcMotor.RunMode runMode) {
        rightFront.setMode(runMode);
        rightBack.setMode(runMode);
        leftFront.setMode(runMode);
        leftBack.setMode(runMode);
    }

    public void setPowers(double power){
        leftFront.setPower(power);
        rightFront.setPower(power);
        leftBack.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        resetAngle();

        if (degrees > 0) {
            degrees -= 1;
        }
        else if (degrees < 0) {
            degrees += 1;
        }

        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        double p = Math.abs(power/degrees);
        double i = (p / 100.0) - 0.00001;

        pidRotate = new PIDController(p, i, 0);

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();

        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_USING_ENCODER);

        if (degrees < 0)
        {
            while (opModeIsActive() && getAngle() == 0)
            {
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(100);
                idle();
            }

            do
            {
                power = pidRotate.performPID(getAngle());
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                idle();
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                idle();
            } while (opModeIsActive() && !pidRotate.onTarget());

        stopBot();

        rotation = getAngle();

        telemetry.addData("global angle", rotation);
        telemetry.update();

        sleep(200);

        resetAngle();
    }

    private void initialization(){
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        up = hardwareMap.dcMotor.get("up");
        linear = hardwareMap.dcMotor.get("linear");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabber = hardwareMap.get(Servo.class, "grabber");
        leftFoundation = hardwareMap.get(Servo.class,"leftFoundation");
        rightFoundation = hardwareMap.get(Servo.class, "rightFoundation");
        capstone = hardwareMap.get(Servo.class, "capstone");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu.initialize(parameters);
        pidDrive = new PIDController(.04, 0, 0);
        pidStrafe = new PIDController(.06, 0.0, 0);

        grabber.setPosition(startPos);

        up.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setPower(1);
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        foundationUnGrab();
        leftDist = hardwareMap.get(DistanceSensor.class, "leftDist");
        rightDist = hardwareMap.get(DistanceSensor.class, "rightDist");
        capstone.setPosition(capGrab);
    }

    private void starting(){
        telemetry.addData("Mode", "running");
        telemetry.update();
        //sleep(500);
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, power);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();
        pidStrafe.setSetpoint(0);
        pidStrafe.setOutputRange(0, power);
        pidStrafe.setInputRange(-90, 90);
        pidStrafe.enable();
        up.setTargetPosition(0);
        up.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        up.setPower(1);
        linear.setTargetPosition(0);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
}


