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

@TeleOp(name="TeleOp1", group="Iterative Opmode")
//@Disabled
public class TeleOp1 extends OpMode
{
    // Declare OpMode members.
    int intakeRotationID = 0;
    boolean clicking = false;
    boolean dpadClick = false;
    boolean armPositinCtrl = false;

    MecanumRobotDrive robot;
    PositionEstimation posEstimation;
    org.firstinspires.ftc.teamcode.samples.PositionControl posControl;

    private final int PICK_POSITION = 0;
    private final int LEVEL1_POSITION = 988;
    private final int LEVEL2_POSITION = 2566;
    private final int LEVEL3_POSITION = 3480;
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
        posEstimation = new PositionEstimation(robot);
        posControl = new PositionControl(robot, posEstimation);

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

        robot.lf.setPower(speeds[0]);
        robot.rf.setPower(speeds[1]);
        robot.lr.setPower(speeds[2]);
        robot.rr.setPower(speeds[3]);

        if(gamepad1.left_bumper){
            if(robot.Arm_E.getCurrentPosition() > 10) {
                double powerGain = Math.min((robot.Arm_E.getCurrentPosition() / 200f), 1);
                robot.Arm_E.setPower(Math.min(-0.6 * powerGain, -0.1));
            }

        }
        else if(gamepad1.right_bumper){
            //please make sure the arm is rise up enough for extend arm
            if (robot.Arm_H.getCurrentPosition() > 1500){
                robot.Arm_E.setPower(0.6);
            }
            else{
                robot.Arm_E.setPower(0.0);
            }

        }
        else{
            robot.Arm_E.setPower(0);
//            robot.Arm_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if(gamepad1.x){
            robot.CM.setPower(-0.4);
        }
        else if(gamepad1.b){
            robot.CM.setPower(0.4);
        }
        else{
            robot.CM.setPower(0);
        }

        if(gamepad1.y){
            if(!clicking){
                if(intakeRotationID != 1){
                    robot.Intake.setPower(0.8);
                    intakeRotationID = 1;
                }
                else{
                    robot.Intake.setPower(0);
                    intakeRotationID = 0;
                }
                clicking = true;
            }
        }
        else if(gamepad1.a){
            if(!clicking){
                if(intakeRotationID != 2){
                    robot.Intake.setPower(-0.8);
                    intakeRotationID = 2;
                }
                else{
                    robot.Intake.setPower(0);
                    intakeRotationID = 0;
                }
                clicking = true;
            }
        }
        else{
            clicking = false;
        }

        telemetry.addData("asc", "aaa " + intakeRotationID);

        //arm_h control here
        //dpad preset position control
        if(gamepad1.dpad_up){
            if(!dpadClick){
                robot.Arm_H.setTargetPosition(LEVEL3_POSITION);
                robot.Arm_H.setPower(1.0);
                robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpadClick = true;
                armPositinCtrl = true;
            }
        }
        else if(gamepad1.dpad_left){
            if(!dpadClick){
                robot.Arm_H.setTargetPosition(LEVEL2_POSITION);
                robot.Arm_H.setPower(1.0);
                robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpadClick = true;
                armPositinCtrl = true;
            }
        }
        else if(gamepad1.dpad_right){
            if(!dpadClick){
                robot.Arm_H.setTargetPosition(LEVEL1_POSITION);
                robot.Arm_H.setPower(1.0);
                robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpadClick = true;
                armPositinCtrl = true;
            }
        }
        else if(gamepad1.dpad_down){
            if(!dpadClick){

                robot.Arm_H.setTargetPosition(PICK_POSITION);
                robot.Arm_H.setPower(1.0);
                robot.Arm_H.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                dpadClick = true;
                armPositinCtrl = true;
            }
        }
        else{
            //robot.Arm_H.setPower(0);
            dpadClick = false;
        }

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




        telemetry.addData("Motor Encoder", "Arm Extend Encoder  = " + (robot.Arm_E.getCurrentPosition()));
        telemetry.addData("Motor Encoder", "Arm Encoder  = " + (robot.Arm_H.getCurrentPosition()));

        double[] position = posEstimation.getRobotPos();
        telemetry.addData("Position", "X = " + position[0] + " | Y: " + position[1]);
        telemetry.update();


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
