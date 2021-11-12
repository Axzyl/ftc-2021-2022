package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumChassis {
    /* Public OpMode members. */
    public DcMotorEx  leftFrontDrive   = null;
    public DcMotorEx  rightFrontDrive  = null;
    public DcMotorEx  leftRearDrive   = null;
    public DcMotorEx  rightRearDrive  = null;
    public DcMotor intake = null;
    public DcMotorEx flyWheelLeft = null;
    public DcMotorEx flyWheelRight = null;
    public DcMotorEx arm = null;
    public Servo claw1 = null;
    public Servo claw2 = null;
    public Servo shootServo = null;
    public Servo blockServo = null;
    public BNO055IMU imu;


    private final double NEW_KP = 10.0;
    private final double NEW_KI = 0.0;
    private final double NEW_KD = 0.0;
    private final double NEW_F = 0.0;


    //private PIDFCoefficients pidNew = new PIDFCoefficients(NEW_P, NEW_I, NEW_D,NEW_F);


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MecanumChassis(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //IMU
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        // Define and Initialize Motors

        leftFrontDrive  = hwMap.get(DcMotorEx.class, "lf");
        rightFrontDrive = hwMap.get(DcMotorEx.class, "rf");
        leftRearDrive  = hwMap.get(DcMotorEx.class, "lr");
        rightRearDrive = hwMap.get(DcMotorEx.class, "rr");

/*
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
*/

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if
        leftRearDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if
        rightRearDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftRearDrive.setPower(0);
        rightRearDrive.setPower(0);

        leftRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRearDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        intake  = hwMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flyWheelLeft = hwMap.get(DcMotorEx.class, "flyWheelLeft");
        flyWheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flyWheelRight = hwMap.get(DcMotorEx.class, "flyWheelRight");
        flyWheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flyWheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flyWheelLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        flyWheelRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        arm = hwMap.get(DcMotorEx.class, "arm");
        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidNew);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armMotor.setPositionPIDFCoefficients(NEW_KP);
        arm.setTargetPositionTolerance(5);

        claw1 = hwMap.get(Servo.class, "handU");
        claw1.setPosition(0.33);
        claw2 = hwMap.get(Servo.class, "handL");
        claw2.setPosition(0.67);


        shootServo = hwMap.get(Servo.class, "shoot");
        shootServo.setPosition(0.68);  //0.7

        blockServo = hwMap.get(Servo.class, "block");
        blockServo.setPosition(0.78);  //0.1

    }


}
