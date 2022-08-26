# FTC-Autonomous-PID

CBlue/CRed and CTeleOp are the autonomous and TeleOp code for the SkyStone FTC game. 

There are three basic steps in the autonomous code - starting, sensing, and finish. It starts by driving up to the blocks. The sensing block senses the skystones using two color sensors. By sensing two blocks in a set of three you can deduce the position of both skystones. The finish block executes different commands to finish the autonomous based on where the skystones are. 

The autonomous utilizes PID controller to ensure the proper direction and number of encoder clicks. It is used in all the methods for strafing and is used so that the robot gradually slows down when it approaches the end of the movement to avoid jerking and sliding. 

The TeleOp code is most interesting for the following mecanum code:
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
    
Using trignometry based on the angle of the control stick, the driver can very simply drive with mecanum wheels. 
