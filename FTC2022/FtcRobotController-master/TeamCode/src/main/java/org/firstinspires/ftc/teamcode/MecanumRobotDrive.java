package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;
import java.util.List;

public class MecanumRobotDrive {

    public DcMotor lf = null;
    public DcMotor lr = null;
    public DcMotor rr = null;
    public DcMotor rf = null;

    public DcMotor Arm_E = null;
    public DcMotor Arm_H = null;
    public DcMotor CM = null;
    public CRServo Intake = null;

    public BNO055IMU imu;
    public ColorSensor colorSensor;

    int initArmE = 0;
    int initArmH = 0;
    int intakeRotationID = 0;
    boolean clicking = false;
    boolean dpadClick = false;
    boolean armMoving = false;

    HardwareMap hardwareMap;

    public MecanumRobotDrive(HardwareMap hw){
        hardwareMap = hw;
        init();
    }

    public void init(){
        lf  = hardwareMap.get(DcMotor.class, "L1");
        lr = hardwareMap.get(DcMotor.class, "L2");
        rf  = hardwareMap.get(DcMotor.class, "R1");
        rr = hardwareMap.get(DcMotor.class, "R2");

        Arm_E  = hardwareMap.get(DcMotor.class, "Arm_E");
        Arm_E.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   //if using position control , need set to brake
        Arm_E.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Arm_H = hardwareMap.get(DcMotor.class, "Arm_H");
        Arm_H.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm_H.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        int arm_H_Position = Arm_H.getCurrentPosition();
        Arm_H.setPower(-1.0);
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        while(Math.abs(Arm_H.getCurrentPosition() - arm_H_Position) > 10){
            arm_H_Position = Arm_H.getCurrentPosition();
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        }
        Arm_H.setPower(0.0);
        Arm_H.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        colorSensor = hardwareMap.colorSensor.get("colour");

        CM  = hardwareMap.get(DcMotor.class, "CM");
        Intake = hardwareMap.get(CRServo.class, "Intake");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Tell the driver that initialization is complete.
        Arm_E.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        initArmE = Arm_E.getCurrentPosition();
        CM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //please keep the robtot still to reset IMU
        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public void Movement(double leftFront, double leftRear, double rightFront, double rightRear){
        lf.setPower(leftFront);
        lr.setPower(leftRear);
        rf.setPower(rightFront);
        rr.setPower(rightRear);
    }
}
