package org.firstinspires.ftc.teamcode.samples;

import android.widget.RadioButton;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name="teleOperation")
@Disabled
public class teleop extends LinearOpMode {
    MecanumChassis robot = new MecanumChassis();
    private volatile Orientation currentAngles;        //used for control
    private volatile double headingAngle ;
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        boolean speed_mode = true;
        boolean aButtonRelease = true;
        //changes the speed for the robot (true is fast)(false is slow)

        /// Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                robot.leftFrontDrive.getCurrentPosition(),
                robot.leftRearDrive.getCurrentPosition(),
                robot.rightFrontDrive.getCurrentPosition(),
                robot.rightRearDrive.getCurrentPosition());

        telemetry.addData("IMU Mode", "IMU calibrating....");
        telemetry.update();

        //make sure the IMU gyro is calibrated before continue
        while(!isStopRequested() && ! robot.imu.isGyroCalibrated() &&
                ! robot.imu.isAccelerometerCalibrated() &&
                ! robot.imu.isMagnetometerCalibrated() &&
                ! robot.imu.isSystemCalibrated())
        {
            idle();
        }

        telemetry.update();

        telemetry.addData("Working Mode", "waiting for start");
        telemetry.update();
        waitForStart();
        //non-refreshing elements

        final int ARM_RESET_POS = 0;
        final int ARM_DOWN_POS = 745;
        final int ARM_HOLD_POS = 449;
        final double SHOOT_1ST = 0.5;
        final double SHOOT_2ND = 0.4;
        final double SHOOT_3RD = 0.25;
        final double SHOOT_RESET = 0.7;
        boolean leftBumper = false;
        boolean leftTrigger = false;
        boolean shooterAct = false;

        int shootCnt = 0;
        int state = 1;
        boolean rightRelease = true;
        boolean gamepadB = false;
        boolean dpadRelease = true;
        //for shoot servo control
        boolean xButtonRelease = true;
        boolean clawFlag = true;

        while (opModeIsActive()){

            //refreshing elements
            //driving variables
            double leftStickXPos = gamepad1.left_stick_x;
            double leftStickYPos = -gamepad1.left_stick_y;
            double rightStickXPos = gamepad1.right_stick_x;

            if (gamepad1.a){
                if (aButtonRelease) {
                    speed_mode = !speed_mode;
                    aButtonRelease = false;
                }
            }
            else{
                aButtonRelease = true;
            }
/*
            if (gamepad1.dpad_right){
                if (rightRelease) {
                    rightRelease = false;
                    state++;
                }
            }
            else{
                rightRelease = true;
            }
*/
            if (speed_mode ) {
                //fast mode
                leftStickXPos = gamepad1.left_stick_x * 0.65;
                leftStickYPos = -gamepad1.left_stick_y * 0.65;
                rightStickXPos = gamepad1.right_stick_x * 0.65;
            } else{
                leftStickXPos = gamepad1.left_stick_x /3.0;
                leftStickYPos = -gamepad1.left_stick_y /3.0;
                rightStickXPos = gamepad1.right_stick_x /3.0;
            }

            robot.leftFrontDrive.setPower(leftStickYPos+leftStickXPos+rightStickXPos);
            robot.leftRearDrive.setPower(leftStickYPos-leftStickXPos+rightStickXPos);
            robot.rightFrontDrive.setPower(leftStickYPos-leftStickXPos-rightStickXPos);
            robot.rightRearDrive.setPower(leftStickYPos+leftStickXPos-rightStickXPos);
            /*
            if (state%2 == 0){
                robot.leftFrontDrive.setPower(leftStickYPos+leftStickXPos+rightStickXPos);
                robot.leftRearDrive.setPower(leftStickYPos-leftStickXPos+rightStickXPos);
                robot.rightFrontDrive.setPower(leftStickYPos-leftStickXPos-rightStickXPos);
                robot.rightRearDrive.setPower(leftStickYPos+leftStickXPos-rightStickXPos);
            }
            else{
                robot.leftFrontDrive.setPower(-(leftStickYPos+leftStickXPos-rightStickXPos));
                robot.leftRearDrive.setPower(-(leftStickYPos-leftStickXPos-rightStickXPos));
                robot.rightFrontDrive.setPower(-(leftStickYPos-leftStickXPos+rightStickXPos));
                robot.rightRearDrive.setPower(-(leftStickYPos+leftStickXPos+rightStickXPos));
            }
            */





            //for claw open/close
            if(gamepad1.x){
                if (xButtonRelease){
                    xButtonRelease = false;
                    if(clawFlag){
                        openClaw();
                        clawFlag = false;
                    }
                    else{
                        closeClaw();
                        clawFlag = true;
                    }
                }

            }
            else{
                xButtonRelease = true;
            }



            //flying wheel
            if (gamepad1.left_bumper){
                if (!leftBumper){
                    shooterAct = !shooterAct;
                    if (shooterAct){
                        robot.flyWheelLeft.setPower(1.0);
                        robot.flyWheelRight.setPower(1.0);
                    }
                    else{
                        robot.flyWheelLeft.setPower(0.0);
                        robot.flyWheelRight.setPower(0.0);
                    }
                    leftBumper = true;
                }
            }

            else if(gamepad1.left_trigger > 0){
                if (!leftTrigger){
                    shooterAct = !shooterAct;
                    if (shooterAct){
                        robot.flyWheelLeft.setPower(0.85);
                        robot.flyWheelRight.setPower(0.85);

                    }
                    else{
                        robot.flyWheelLeft.setPower(0.0);
                        robot.flyWheelRight.setPower(0.0);
                    }
                    leftTrigger = true;
                }
            }
            else{
                leftBumper = false;
                leftTrigger = false;
            }

            //for separate shooting, 1,2,3
            if (gamepad1.b) {
                if (!gamepadB){
                    gamepadB = true;
                    shootCnt ++;
                    if (shootCnt == 1){
                        robot.shootServo.setPosition(SHOOT_1ST);
                    }
                    else if(shootCnt == 2){
                        robot.shootServo.setPosition(SHOOT_2ND);
                    }
                    else if(shootCnt == 3){
                        shootCnt = 0;
                        robot.shootServo.setPosition(SHOOT_3RD);
                        sleep(500);
                        robot.shootServo.setPosition(SHOOT_RESET);
                    }
                }
            }
            else {
                gamepadB = false;
            }
            //for 3 shooting in once
            if (gamepad1.y){
                int timeCnt = 0;
                double servoStep = (0.7- 0.25) /50; //   //0.7s  -- 0.25
                double servoPosition = SHOOT_RESET;
                int delayTime = 10;
                while(timeCnt < 52){
                    timeCnt ++;
                    servoPosition= SHOOT_RESET - servoStep * timeCnt;
                    if (servoPosition < SHOOT_3RD) servoPosition = SHOOT_3RD;
                    robot.shootServo.setPosition(servoPosition);
                    delayTime = 10 * (timeCnt /10);     // slow down with second and third ring
                    if (delayTime > 30) delayTime = 30;
                    sleep(delayTime);
                }

                /*
                robot.shootServo.setPosition(SHOOT_2ND);
                sleep(700);
                robot.shootServo.setPosition(SHOOT_3RD);
                sleep(300);*/
                robot.shootServo.setPosition(SHOOT_RESET);
                shootCnt = 0;
            }

            //intake
            if (!shooterAct){
                if (gamepad1.right_trigger > 0){
                    robot.intake.setPower(-1);
                }
                else if (gamepad1.right_bumper) {
                    robot.intake.setPower(1);

                } else {
                    robot.intake.setPower(0);
                }
            }
            else{
                robot.intake.setPower(0);
            }

            //for arm control

            if (gamepad1.dpad_up){
                if (dpadRelease) {
                    robot.arm.setTargetPosition(ARM_RESET_POS);
                    robot.arm.setPower(0.6);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    dpadRelease = false;
                }

            }
            else if(gamepad1.dpad_down){
                if (dpadRelease) {
                    robot.arm.setTargetPosition(ARM_DOWN_POS);
                    robot.arm.setPower(0.6);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    dpadRelease = false;
                }
            }
            else if(gamepad1.dpad_left){
                if (dpadRelease) {
                    robot.arm.setTargetPosition(ARM_HOLD_POS);
                    robot.arm.setPower(1);
                    robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                     dpadRelease= false;
                }
            }
            else if(gamepad1.dpad_right){
                if (dpadRelease) {
                    turnToGoal();
                    dpadRelease= false;
                }
            }

            else{
                dpadRelease = true;
            }

            robot.blockServo.setPosition(0.1);
            this.currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            this.headingAngle = AngleUnit.DEGREES.fromUnit(this.currentAngles.angleUnit, this.currentAngles.firstAngle);

            // Debug
            telemetry.addData("clawFlag", clawFlag);
            telemetry.addData("armPos", robot.arm.getCurrentPosition());
            telemetry.addData("SpeedMode", speed_mode);
            telemetry.addData("wheelSpeed", robot.flyWheelLeft.getVelocity());
            telemetry.addData("Heading", this.headingAngle);
            telemetry.update();

        }
    }
    private void closeClaw(){
        robot.claw1.setPosition(0.21);
        robot.claw2.setPosition(0.79);
    }
    private void openClaw(){
        robot.claw1.setPosition(0.67);
        robot.claw2.setPosition(0.33);
    }

    private void turnToGoal(){
        float targetAngle = 0;
        double error = 0;
        double kp = 0.01;
        double power = 0;
        while(true){
            this.currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            this.headingAngle = AngleUnit.DEGREES.fromUnit(this.currentAngles.angleUnit, this.currentAngles.firstAngle);
            error = this.headingAngle - targetAngle;
            if (Math.abs(error) < 1.0){
                driveRobot(0,0, 0, 0 );
                break;
            }
            else{
                if (Math.abs(error) < 10) {
                    //in 10 degree error range, keep slow speed
                    power = Math.signum(error) * 0.08;
                }
                else{
                    power = error * kp;
                }

                if (Math.abs(power) > 0.5)  power = 0.5 * Math.signum(power);
                if (Math.abs(power) < 0.1)  power = 0.1 * Math.signum(power);
                driveRobot(power,-power, power, -power );
            }
        }
    }

    private void driveRobot(double leftPowerF, double rightPowerF, double leftPowerR, double rightPowerR){
        //here if we use setPower function, we need normalize the speed
        //or may use the setVelocity() function, need transform to radian
        /*
        leftPowerF = (Math.abs(leftPowerF) < 0.1? Math.signum(leftPowerF) * 0.1: leftPowerF);
        leftPowerR = (Math.abs(leftPowerR) < 0.1? Math.signum(leftPowerR) * 0.1: leftPowerR);
        rightPowerF = (Math.abs(rightPowerF) < 0.1? Math.signum(rightPowerF) * 0.1: rightPowerF);
        rightPowerR = (Math.abs(rightPowerR) < 0.1? Math.signum(rightPowerR) * 0.1: rightPowerR);
*/
        leftPowerF    = Range.clip(leftPowerF , -1.0, 1.0) ;
        rightPowerF   = Range.clip(rightPowerF, -1.0, 1.0) ;
        leftPowerR    = Range.clip(leftPowerR, -1.0, 1.0) ;
        rightPowerR   = Range.clip(rightPowerR , -1.0, 1.0) ;
        this.robot.rightFrontDrive.setPower(rightPowerF);
        this.robot.rightRearDrive.setPower(rightPowerR);
        this.robot.leftRearDrive.setPower(leftPowerR);
        this.robot.leftFrontDrive.setPower(leftPowerF);
    }
}