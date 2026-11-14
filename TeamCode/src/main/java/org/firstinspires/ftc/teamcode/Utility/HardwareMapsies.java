package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareMapsies {

    ///////////////////////////
    /// INTEGRATED HARDWARE ///
    ///////////////////////////

    // Hardware Map //

    HardwareMap hardwareMap;

    // IMU //

    BNO055IMU imu;

    //////////////
    /// MOTORS ///
    //////////////

    // Drivetrain Motors //

    public DcMotor FL;
    public DcMotor FR;
    public DcMotor BL;
    public DcMotor BR;

    public DcMotor launchingMotor;
    public DcMotor intakeMotor;

    public Servo holdServo;
    public Servo flowerServo;



    ///////////////////
    /// I2C DEVICES ///
    ///////////////////

    // Pinpoint //
    public GoBildaPinpointDriver pinpoint;

    //////////////
    /// WEBCAM ///
    //////////////

    public static final int VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY = 0x046D;

    public static final int PRODUCT_ID_ARDUCAM_OV5648 = 0x0825;



    //////////////////////
    /// INITIALIZATION ///
    //////////////////////

    /// IMU

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /// MISC INIT

    public void init (HardwareMap i){
        hardwareMap = i;

        // There are several "ZeroPowerBehaviors", usually you want the robot to brake when the motors aren't powered, so as to resist motion

        FL = initMotor("FL", true, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR = initMotor("FR", false, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL = initMotor("BL", true, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR = initMotor("BR", false, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        holdServo = i.get(Servo.class, "holdServo");
        flowerServo = i.get(Servo.class, "flowerServo");

        launchingMotor = initMotor("launchingMotor", false, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor = initMotor("intakeMotor", false, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER);

//        launcher = initMotor("launcher", false, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER);

        // Pinpoint is our odometry method, we use it to get the movement of our robot on the field
        pinpoint = i.get(GoBildaPinpointDriver.class, "pinpoint");

    }

    /// MOTOR INIT

    public DcMotor initMotor(String name, boolean isReverse, DcMotor.ZeroPowerBehavior zeroPowerBehavior, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        if (isReverse) {
            motor.setDirection(DcMotor.Direction.REVERSE);
        } else {
            motor.setDirection(DcMotor.Direction.FORWARD);
        }

        motor.setPower(0.0);
        motor.setZeroPowerBehavior(zeroPowerBehavior);
        motor.setMode(runMode);

        return motor;
    }

    /////////////////////
    /// MOTOR HELPERS ///
    /////////////////////


    /// ROTATIONAL PID

    // TODO: Make rotational correction PID!

    /////////////////////
    /// ANGLE HELPERS ///
    /////////////////////


    /*
    Some Math library functions get mad if you have an angle that's too big (i.e, >2pi or <-2pi)
    This function adds or subtracts 2pi until it reaches the same angle, but within that range
     */
    public static double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;
    }

}