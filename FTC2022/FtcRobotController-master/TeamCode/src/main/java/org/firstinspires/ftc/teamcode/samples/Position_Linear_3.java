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

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.MecanumRobotDrive;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;

// 2021 game, autonomous code

@Autonomous(name="AutoCompetition3", group="Linear Opmode")
//@Disabled
public class Position_Linear_3 extends LinearOpMode {
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
    boolean threadFinished = false;
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

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2, 16.0/9.0);
        }

        //first detect object position
        int markerCnt = 0;
        int duckCnt = 0;
        float[] markerX = new float[2];
        float[] duckX = new float[2];
        boolean detectFlag = false;
        int posFlag = 1;
        List<Float> markers = new ArrayList<Float>();
        while(!isStarted()){
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                markers.clear();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;

                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        if (recognition.getLabel().equals("Marker")) {
                            markers.add(recognition.getRight());
                            //detectFlag = true;
                        }
//                        else if (recognition.getLabel().equals("Duck")) {
//                            duckX[duckCnt] = recognition.getRight();
//                            duckCnt++;
//                            detectFlag = true;
//                        }
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        i++;
                    }
                }
            }

            if (markers.size() == 1){
                if (markers.get(0) > 700){
                    posFlag = 1;
                }
                else{
                    posFlag = 2;
                }
            }
            else{
                posFlag = 3;
            }

            telemetry.addData("Status", "Initialized");

            telemetry.addData(">", "Press Play to start autonomous program");
            telemetry.addData(">", "Position: " + posFlag);
            telemetry.update();
        }



        // Wait for the game to start (driver presses PLAY)
        runtime.reset();

//        telemetry.addData("Detect Position:", "%d", posFlag);
        telemetry.update();
        // understanding coordinate system
        //              Robot Front
        //                  ^ (X+)
        //                  |
        //                  |
        //Robot Left(Y+)    |       Robot Right(Y-)
        //<-----------------------------------------
        //                  |
        //                  |(X-)
        posFlag = 3;  //just for debug

        if(!opModeIsActive()) stop();

        if (posFlag == 1) {
            robot.Arm_H.setTargetPosition(LEVEL1_POSITION);
            robot.Arm_H.setPower(1.0);
            robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //while(robot.Arm_H.getCurrentPosition() < LEVEL1_POSITION - 1600){}
            robot.Arm_E.setTargetPosition(600);
            robot.Arm_E.setPower(0.6);
            robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Log.d("PosFlag","1");
            goToWayPoint(0.44,0.1, 0.0,0.8,90, 0.05,3,1.5);
            robot.Movement(0,0,0,0);
            robot.Intake1.setPower(-0.8);
            robot.Intake2.setPower(-0.8);
        }
        if (posFlag == 2) {
            robot.Arm_H.setTargetPosition(LEVEL2_POSITION);
            robot.Arm_H.setPower(1.0);
            robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.Arm_H.getCurrentPosition() < LEVEL2_POSITION - 2400){}
            robot.Arm_E.setTargetPosition(700);
            robot.Arm_E.setPower(0.6);
            robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Log.d("PosFlag","2");
            goToWayPoint(0.40,0.1, 0.0,0.8,90, 0.05,3,1.5);
            robot.Movement(0,0,0,0);
            robot.Intake1.setPower(-0.8);
            robot.Intake2.setPower(-0.8);
        }
        if (posFlag == 3) {
            robot.Arm_H.setTargetPosition(LEVEL3_POSITION);
            robot.Arm_H.setPower(1.0);
            robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(robot.Arm_H.getCurrentPosition() < LEVEL3_POSITION - 2400){}
            robot.Arm_E.setTargetPosition(800);
            robot.Arm_E.setPower(0.6);
            robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Log.d("PosFlag","3");
            goToWayPoint(0.50,0.1, 0.0,0.8,90, 0.05,3,1.5);
            robot.Movement(0,0,0,0);
            robot.Intake1.setPower(-0.8);
            robot.Intake2.setPower(-0.8);
        }

        Thread.sleep(1000);
        robot.Intake1.setPower(0);
        robot.Intake2.setPower(0);
        robot.Arm_E.setTargetPosition(0);
        robot.Arm_E.setPower(0.6);
        robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goToWayPoint(0.18,1.4, -90,2,180, 0.02,1,3.5);
        robot.Movement(0.13,-0.35,-0.35,0.13);

        Thread.sleep(750);
        robot.Movement(0,0,0,0);
        //positionEstimation.ResetAll();
        //goToWayPoint(-0.42,0, -90,0.8,90, 0.02,1);
        robot.CM.setPower(-0.23);
        Thread.sleep(2750);
        robot.CM.setPower(0);

        robot.Intake1.setPower(1);
        robot.Intake2.setPower(1);
        robot.Arm_H.setTargetPosition(PICK_POSITION);
        robot.Arm_H.setPower(1.0);
        robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        goToWayPoint(0.30,1.08, -90,2.25,90, 0.08,3,2);
        goToWayPoint(-0.05,0.20, -90,2.25,90, 0.03,3,2);
        goToWayPoint(-0.13,-0.94, -93,2.25,90, 0.03,1,2);

        robot.Movement(0.2,-0.8,-0.8,0.2);
        Thread.sleep(350);
        robot.Movement(0,0,0,0);
        //goToWayPoint(-0.18,-0.95, -90,1.5,90, 0.02,1,5);

        threadFinished = false;
        DropShipmentV2();
        while (!threadFinished){}

        goToWayPoint(robotPos[0] - 0.07, robotPos[1] - 0.90, RadtoDeg(robotPos[2]), 2.25, 180, 0.1, 8,2);

        Thread.sleep(250);

        robot.Movement(0.2,-0.8,-0.8,0.2);
        Thread.sleep(600);
        robot.Movement(0,0,0,0);

        //goToWayPoint(-0.18,-0.95, -90,1.5,90, 0.02,1,5);

        threadFinished = false;
        DropShipmentV2();
        while (!threadFinished){}

        goToWayPoint(robotPos[0] - 0.07, robotPos[1] - 0.66, RadtoDeg(robotPos[2]), 2.25, 180, 0.1, 8,2);

        robot.Movement(0.4,-0.4,-0.4,0.4);
        Thread.sleep(750);
        robot.Movement(0,0,0,0);


        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            robotPos = positionEstimation.getRobotPos();
            telemetry.addData("Detect Position",  "at %d",posFlag);
            telemetry.addData("RobotPos",  "at %5f :%5f:%5f",
                    robotPos[0], robotPos[1], robotPos[2] * 180 / Math.PI);
            telemetry.addData("Debug",  "at %5f",
                    positionControl.debug_task());
            if (this.positionControl.checkTaskDone()){
                telemetry.addData("Task", "Done");
            }
            telemetry.update();
        }


        this.positionControl.stopRobot();
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
        while(!this.positionControl.checkTaskDone() && a < timeLimit * 40 && opModeIsActive()){
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
        positionControl.SetTaskDone();
        Thread.sleep(50);
    }

    private void goToWayPoint(double x, double y, double angle, double vel, double vw, double disRes, double angleRes, double timeLimit, boolean brake) throws InterruptedException {
        targetPos[0] = x;//1.5;  //x
        targetPos[1] = y;//-0.6;   //y
        targetPos[2] = angle * Math.PI / 180; // Math.PI /2;   //heading, radian
        this.positionControl.goToTargetPosition(targetPos, vel,vw * Math.PI / 180, disRes,angleRes,brake);
        float a = 0;
        while(!this.positionControl.checkTaskDone() && a < timeLimit * 40 && opModeIsActive()){
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
        positionControl.SetTaskDone();
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

    void DropShipmentV2() throws InterruptedException {

        goToWayPoint(robotPos[0] - 0.05, robotPos[1] + 0.10, RadtoDeg(robotPos[2]), 2, 90, 0.04, 1,3);

        robot.Movement(0,0,0,0);
        robot.Arm_H.setTargetPosition(3100);
        robot.Arm_H.setPower(1.0);
        robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmExtendDelay(500,2400);

        while(robot.colorSensor.red() < 300){
            robot.Movement(-0.4,-0.6, -0.6, -0.4);
        }

        goToWayPoint(robotPos[0], robotPos[1] + 0.47, RadtoDeg(robotPos[2]), 2.25, 90, 0.1, 1,3);

        goToWayPoint(robotPos[0] + 0.10, robotPos[1] + 0.6, RadtoDeg(robotPos[2]) + 90, 2.25, 120, 0.04, 2,1.75);

        robot.Intake1.setPower(-0.8);
        robot.Intake2.setPower(-0.8);
        Thread.sleep(500);
        robot.Intake1.setPower(0);
        robot.Intake2.setPower(0);
        robot.Arm_E.setTargetPosition(0);
        robot.Arm_E.setPower(1);
        robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmDelay(800,PICK_POSITION);
        goToWayPoint(robotPos[0] - 0.15, robotPos[1] - 0.50, RadtoDeg(robotPos[2]) - 90, 2.25, 360, 0.1, 4,3);
        robot.Intake1.setPower(1.0);
        robot.Intake2.setPower(1.0);

//                    robot.Movement(0.6,0.8,0.8,0.6);
//                    Thread.sleep(800);
//                    robot.Movement(0,0,0,0);
        threadFinished = true;
        positionControl.InterruptThread();


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

    void ArmExtendDelay(long delay, int position){
        new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Thread.sleep(delay);
                    robot.Arm_E.setTargetPosition(position);
                    robot.Arm_E.setPower(0.8);
                    robot.Arm_E.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
