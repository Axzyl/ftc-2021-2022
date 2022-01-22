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

package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.MecanumRobotDrive;

import java.util.ArrayList;
import java.util.List;

// 2021 game, autonomous code

@Autonomous(name="Testing", group="Linear Opmode")
//@Disabled
public class TestAndStuff extends LinearOpMode {
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforia LicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "ARSsLcv/////AAABmaPnuXwWvUUcmEwKRJUD7zsqO7JIqriiHhFyZBocTTTMF8T8EA4mCbJtMqnxh1TufzQXUOapLLMgLOG9+pJ77k4LT2uFLHXqlvu6yEXSpXpYi2xtaMfGHYOnxiDtXXjp+1BUc/jZBGgET0URPPPu1HXwGy8MSHS5PDM7ZlZobnMSAuHZFKjue5KYUHBHe4QBbZ1/S9ybpA33GNHpcwK3NPAI0jeXkrovdvBDq0fE56lMN7xTsGOKcQWf8KdpKhWdvS9lzd2u1mGbNontyiXJVyKSC5E7Vr4wszt68uiSCPEy4kWZ0eh+5S0MmgJZprRo7SX4s9tSXCmuU+bceuu2X8kXZAzwX78EtkqrEe1bm3O+";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    MecanumRobotDrive robot;   //
    private ElapsedTime runtime = new ElapsedTime();
    private PositionEstimation positionEstimation = null;
    private PositionControl positionControl = null;
    private volatile double[] robotPos = new double[3];
    private volatile double[] targetPos = new double[3];
    private final int PICK_POSITION = 0;
    private final int LEVEL1_POSITION = 1128;
    private final int LEVEL2_POSITION = 2300;
    private final int LEVEL3_POSITION = 3480;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MecanumRobotDrive(hardwareMap);
        while(!robot.imu.isAccelerometerCalibrated() &&
                !robot.imu.isGyroCalibrated() &&
                !robot.imu.isMagnetometerCalibrated() &&
                !robot.imu.isSystemCalibrated()){
            Thread.yield();
        }
        this.positionEstimation = new PositionEstimation(robot);
        this.positionControl = new PositionControl(robot, this.positionEstimation);
        this.positionControl.stopRobot();

        waitForStart();

        robot.wingServo.setPosition(0.6);



    }

    /****************************************************************************
     *
     * @param x: position x value, unit: m
     * @param y: position y value, unit: m
     * @param angle: position orientation, unit : degree
     * @param vel: linear velocity, unit: m/s
     * @param vw: turning velocity, unit: degree/s, 90, means turn 90 degree in one second
     * @param disRes: position(x,x) resolution, for example, 0.02, it means in 2cm radius of target, the position control is done
     * @param angleRes: orientation control resolution, for example, 5, it means in 5 degree error, the control is done
     * @throws InterruptedException
     */

    private void goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes, double timeLimit) throws InterruptedException {
        targetPos[0] = x;//1.5;  //x
        targetPos[1] = y;//-0.6;   //y
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes);
        float a = 0;
        while(!this.positionControl.checkTaskDone() && a < timeLimit * 40){
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0], robotPos[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
            }
            telemetry.update();
            a++;
            Thread.sleep(25);
        }
        //positionControl.SetTaskDone();
        Thread.sleep(50);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.6f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    void DropShipmentV2(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {

                    goToWayPoint(robotPos[0] - 0.05, robotPos[1] + 0.15, RadtoDeg(robotPos[2]), 2, 90, 0.04, 1,3);

                    while(robot.colorSensor.red() < 600){
                        robot.Movement(-0.8,-0.8, -0.8, -0.6);
                    }
                    robot.Movement(0,0,0,0);
                    robot.Arm_H.setTargetPosition(3100);
                    robot.Arm_H.setPower(1.0);
                    robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.Arm_E.setTargetPosition(2400);
                    robot.Arm_E.setPower(0.8);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    goToWayPoint(robotPos[0], robotPos[1] + 0.47, RadtoDeg(robotPos[2]), 2.5, 90, 0.1, 1,3);

                    goToWayPoint(robotPos[0] + 0.10, robotPos[1] + 0.57, RadtoDeg(robotPos[2]) + 90, 2.5, 360, 0.02, 1,3);

                    robot.Intake1.setPower(-0.8);
                    robot.Intake2.setPower(-0.8);
                    Thread.sleep(500);
                    robot.Intake1.setPower(0);
                    robot.Intake2.setPower(0);
                    robot.Arm_E.setTargetPosition(0);
                    robot.Arm_E.setPower(1);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmDelay(800,PICK_POSITION);
                    goToWayPoint(robotPos[0] - 0.15, robotPos[1] - 0.50, RadtoDeg(robotPos[2]) - 90, 2.5, 360, 0.1, 4,3);
                    robot.Intake1.setPower(1.0);
                    robot.Intake2.setPower(1.0);
                    goToWayPoint(robotPos[0] - 0.05, robotPos[1] - 0.78, RadtoDeg(robotPos[2]), 2.5, 180, 0.1, 4,3);


                    positionControl.InterruptThread();

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


}
