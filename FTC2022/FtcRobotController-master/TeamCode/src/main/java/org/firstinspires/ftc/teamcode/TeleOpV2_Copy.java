/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.samples.PositionControl;
import org.firstinspires.ftc.teamcode.samples.PositionEstimation;

import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOpV2-Experimental", group="Iterative Opmode")
//@Disabled
public class TeleOpV2_Copy extends OpMode
{
    // Declare OpMode members.
    int intakeRotationID = 0;
    boolean clicking = false;
    boolean dpadClick = false;
    boolean armPositinCtrl = false;
    double speedMultiplier = 0.8;
    int CMTogglePos = 0;
    boolean CMClicking = false;
    boolean runningThread = false;

    private volatile double[] robotPos = new double[3];
    private volatile double[] targetPos = new double[3];

    MecanumRobotDrive robot;
    PositionEstimation positionEstimation;
    org.firstinspires.ftc.teamcode.samples.PositionControl positionControl;

    private final int PICK_POSITION = 0;
    private final int LEVEL1_POSITION = 988;
    private final int LEVEL2_POSITION = 2566;
    private final int LEVEL3_POSITION = 3480;

    private ElapsedTime runtime = new ElapsedTime();

    private boolean btnStartRelease = true;
    private int hookPosID = 0;

    double offsetX = 0;
    double offsetY = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        robot = new MecanumRobotDrive(hardwareMap);
        telemetry.addData("Status", "Initialized");
        positionEstimation = new PositionEstimation(robot);
        positionControl = new PositionControl(robot, positionEstimation);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.

        ////double drive = -gamepad1.left_stick_y;
        ////double turn  =  gamepad1.right_stick_x;


        ////leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        ////rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        robotPos = positionEstimation.getRobotPos();

        if(!runningThread){

            double drive  = Math.pow(-gamepad1.left_stick_y,3);
            double strafe = Math.pow(gamepad1.left_stick_x,3);
            double twist  = Math.pow(gamepad1.right_stick_x,5);

            //may need set 2 level maximum speed
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            double max = Math.abs(speeds[0]);
            for(int i = 0; i < speeds.length; i++) {
                if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
            }

            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            if(gamepad1.left_stick_button || gamepad1.right_stick_button) speedMultiplier = 0.4;

            if(gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0 && gamepad1.right_stick_y == 0)
                speedMultiplier = 0.8;

            robot.lf.setPower(speeds[0] * speedMultiplier);
            robot.rf.setPower(speeds[1] * speedMultiplier);
            robot.lr.setPower(speeds[2] * speedMultiplier);
            robot.rr.setPower(speeds[3] * speedMultiplier);

            if(gamepad1.left_bumper){
                if(robot.Arm_E.getCurrentPosition() > 10) {
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    double powerGain = Math.min((robot.Arm_E.getCurrentPosition() / 200f), 1);
                    robot.Arm_E.setPower(Math.min(-0.8 * powerGain, -0.1));
                }

            }
            else if(gamepad1.right_bumper){
                //please make sure the arm is rise up enough for extend arm
                if (robot.Arm_H.getCurrentPosition() > 1500){
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_E.setPower(0.8);
                }
                else{
                    robot.Arm_E.setPower(0.0);
                }

            }
            else{
                robot.Arm_E.setPower(0);
//            robot.Arm_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            if(gamepad1.b){
                if(!CMClicking){
                    CMTogglePos += 1;
                    if(CMTogglePos == 2) CMTogglePos = 0;
                    CMClicking = true;
                }
            }
            else{
                CMClicking = false;
            }

            if(CMTogglePos == 0) robot.CM.setPower(0);
            else if(CMTogglePos == 1) robot.CM.setPower(-0.25);

            telemetry.addData("SensorData: ", robot.colorSensor.red() + " " + robot.colorSensor.green() + " " + robot.colorSensor.blue());
//        telemetry.addData("asc", "aaa " + intakeRotationID);

            //arm_h control here
            //dpad preset position control


            //left/right trigger button manually control, need set position to protect
            if(gamepad1.left_trigger > 0.5){
                if (robot.Arm_H.getCurrentPosition() < 3800){
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_H.setPower(1.0);
                }
                else{
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_H.setPower(0.0);
                }
                armPositinCtrl = false;
            }
            else if (gamepad1.right_trigger > 0.5){
                if (robot.Arm_H.getCurrentPosition() > 5){
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_H.setPower(-1.0);
                }
                else{
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_H.setPower(0.0);
                }
                armPositinCtrl = false;
            }
            else{
                if (!armPositinCtrl) {
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.Arm_H.setPower(0);
                }
            }
        }

        if(gamepad1.x){
            if(!clicking && !runningThread){
                if(intakeRotationID != 1){
                    robot.Intake1.setPower(1.0);
                    robot.Intake2.setPower(1.0);
                    intakeRotationID = 1;
                }
                else{
                    robot.Intake1.setPower(0);
                    robot.Intake2.setPower(0);
                    intakeRotationID = 0;
                }
                clicking = true;
            }
        }
        else if(gamepad1.a) {
            if (!clicking && !runningThread) {
                if (intakeRotationID != 2) {
                    robot.Intake1.setPower(-0.8);
                    robot.Intake2.setPower(-0.8);
                    intakeRotationID = 2;
                } else {
                    robot.Intake1.setPower(0);
                    robot.Intake2.setPower(0);
                    intakeRotationID = 0;
                }
                clicking = true;
            }
        }
        else if(gamepad1.y){
            if(!clicking && !runningThread){
                DropShipmentV2();
                clicking = true;
            }

            if(!clicking && runningThread){
                clicking = true;
                try {
                    throw new InterruptedException("Interrupt");
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }
        else{
            clicking = false;
        }

        if (gamepad1.start){
            if (btnStartRelease){
                btnStartRelease = false;
                if (hookPosID != 1){
                    robot.hookServo.setPosition(0.75);
                    hookPosID = 1;
                }
                else{
                    hookPosID = 2;
                    robot.hookServo.setPosition(0.5);
                }
            }
            //open
        }
        else{
            btnStartRelease = true;
        }

        if(gamepad1.dpad_up){
            if(!dpadClick){
                if(!runningThread){
                    robot.Arm_H.setTargetPosition(LEVEL3_POSITION);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dpadClick = true;
                    armPositinCtrl = true;
                }
                else{
                    offsetY += 0.2f;
                }

            }
        }
        else if(gamepad1.dpad_left){
            if(!dpadClick){
                if(!runningThread) {
                    robot.Arm_H.setTargetPosition(LEVEL2_POSITION);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dpadClick = true;
                    armPositinCtrl = true;
                }
                else{
                    offsetX -= 0.2f;
                }
            }
        }
        else if(gamepad1.dpad_right){
            if(!dpadClick){
                if(!runningThread) {
                    robot.Arm_H.setTargetPosition(LEVEL1_POSITION);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dpadClick = true;
                    armPositinCtrl = true;
                }
                else{
                    offsetX += 0.2f;
                }
            }
        }
        else if(gamepad1.dpad_down){
            if(!dpadClick){
                if(!runningThread) {
                    robot.Arm_H.setTargetPosition(PICK_POSITION);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    dpadClick = true;
                    armPositinCtrl = true;
                }
                else{
                    offsetY -= 0.2f;
                }
            }
        }
        else{
            //robot.Arm_H.setPower(0);
            dpadClick = false;
        }

        telemetry.addData("Motor Encoder", "Arm Extend Encoder  = " + (robot.Arm_E.getCurrentPosition()));
//        telemetry.addData("Motor Encoder", "Arm Encoder  = " + (robot.Arm_H.getCurrentPosition()));
        telemetry.addData("Color",  "at %d:%d:%d",robot.colorSensor.red(), robot.colorSensor.green(), robot.colorSensor.blue());
        telemetry.addData("Block",  "at %d:%d:%d",robot.blockSensor.red(), robot.blockSensor.green(), robot.blockSensor.blue());
//        telemetry.addData("Position", "X = " + robotPos[0] + " | Y: " + robotPos[1] + " | Rot: " + robotPos[2]);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


    }

    void DropShipment(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while(gamepad1.y){}
                    runningThread = true;
                    armPositinCtrl = true;
                    if(goToWayPoint(robotPos[0] - 0.15, robotPos[1]-0.05, RadtoDeg(robotPos[2]), 2, 90, 0.04, 1,3, true)){
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    while(robot.colorSensor.red() < 600){
                        robot.Movement(-0.8,-0.8, -0.8, -0.6);
                        if(gamepad1.y){
                            runningThread = false;
                            armPositinCtrl = false;
                            return;
                        }
                    }
                    robot.Movement(0,0,0,0);
                    robot.Arm_H.setTargetPosition(3100);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(goToWayPoint(robotPos[0] - 0.94, robotPos[1], RadtoDeg(robotPos[2]), 2.5, 90, 0.02, 1,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    robot.Arm_E.setTargetPosition(2400);
                    robot.Arm_E.setPower(0.8);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(goToWayPoint(robotPos[0], robotPos[1] + 0.20, RadtoDeg(robotPos[2]) + 90, 2.5, 180, 0.02, 4,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    robot.Intake1.setPower(-0.8);
                    robot.Intake2.setPower(-0.8);
                    Thread.sleep(800);
                    robot.Intake1.setPower(0);
                    robot.Intake2.setPower(0);
                    robot.Arm_E.setTargetPosition(10);
                    robot.Arm_E.setPower(0.8);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(goToWayPoint(robotPos[0], robotPos[1] - 0.20, RadtoDeg(robotPos[2]) - 90, 2.5, 360, 0.02, 4,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    robot.Arm_H.setTargetPosition(PICK_POSITION);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    if(goToWayPoint(robotPos[0] + 1, robotPos[1] - 0.05, RadtoDeg(robotPos[2]), 2.5, 180, 0.1, 4,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    robot.Intake1.setPower(1.0);
                    robot.Intake2.setPower(1.0);
                    positionControl.InterruptThread();
                    runningThread = false;
                    armPositinCtrl = false;
                } catch (InterruptedException e) {

                }
            }
        }).start();
    }

    void DropShipmentV2(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    while(gamepad1.y){}
                    runningThread = true;
                    armPositinCtrl = true;
                    if(goToWayPoint(robotPos[0] - 0.15, robotPos[1]-0.05, RadtoDeg(robotPos[2]), 2, 90, 0.04, 1,3, true)){
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    while(robot.colorSensor.red() < 600){
                        robot.Movement(-0.8,-0.8, -0.8, -0.6);
                        if(gamepad1.y){
                            runningThread = false;
                            armPositinCtrl = false;
                            return;
                        }
                    }
                    robot.Movement(0,0,0,0);
                    robot.Arm_H.setTargetPosition(3100);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.Arm_E.setTargetPosition(2400);
                    robot.Arm_E.setPower(0.8);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    if(goToWayPoint(robotPos[0] - 0.47, robotPos[1], RadtoDeg(robotPos[2]), 2.5, 90, 0.1, 1,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    if(goToWayPoint(robotPos[0] - 0.57, robotPos[1] + 0.10, RadtoDeg(robotPos[2]) + 90, 2.5, 180, 0.02, 1,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }

                    robot.Intake1.setPower(-0.8);
                    robot.Intake2.setPower(-0.8);
                    Thread.sleep(500);
                    robot.Intake1.setPower(0);
                    robot.Intake2.setPower(0);
                    robot.Arm_E.setTargetPosition(0);
                    robot.Arm_E.setPower(1);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmDelay(800,PICK_POSITION);
                    if(goToWayPoint(robotPos[0] + 0.50, robotPos[1] - 0.20, RadtoDeg(robotPos[2]) - 90, 2, 360, 0.1, 4,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    if(goToWayPoint(robotPos[0] + 0.54, robotPos[1] - 0.10, RadtoDeg(robotPos[2]), 2.5, 180, 0.1, 4,3, true)) {
                        runningThread = false;
                        armPositinCtrl = false;
                        return;
                    }
                    robot.Intake1.setPower(1.0);
                    robot.Intake2.setPower(1.0);
                    positionControl.InterruptThread();
                    runningThread = false;
                    armPositinCtrl = false;
                } catch (InterruptedException e) {

                }
            }
        }).start();
    }

    void ArmDelay(long delay, int position){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Thread.sleep(delay);
                    robot.Arm_H.setTargetPosition(position);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

            }
        }).start();
    }

    double RadtoDeg(double rad){
        return rad / (2 * Math.PI) * 360;
    }

    private boolean goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes, double timeLimit, boolean thread) throws InterruptedException {
        targetPos[0] = x;//1.5;  //x
        targetPos[1] = y;//-0.6;   //y
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes);
        float a = 0;
        while(!this.positionControl.checkTaskDone() && a < timeLimit * 40){
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0], robotPos[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
                break;
            }
            telemetry.update();
            a++;

            if(gamepad1.y){
                positionControl.SetTaskDone();
                return true;
            }

            Thread.sleep(25);
        }
        positionControl.SetTaskDone();
        Thread.sleep(50);
        return false;
    }

    private boolean goToWayPointOffset(double x, double y, double angle, double vel, double vw, double disRes, double angleRes, double timeLimit, boolean thread) throws InterruptedException {
        targetPos[0] = x + offsetX;//1.5;  //x
        targetPos[1] = y + offsetY;//-0.6;   //y
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        double[] currentOffset = {offsetX, offsetY};
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes);
        float a = 0;
        while(!this.positionControl.checkTaskDone() && a < timeLimit * 40){
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0] + offsetX - currentOffset[0], robotPos[1] + offsetY - currentOffset[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
                break;
            }
            telemetry.update();
            a++;

            if(gamepad1.y){
                positionControl.SetTaskDone();
                return true;
            }

            Thread.sleep(25);
        }
        positionControl.SetTaskDone();
        Thread.sleep(50);
        return false;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}