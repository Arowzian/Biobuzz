package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utility.HardwareMapsies;
import org.firstinspires.ftc.teamcode.Utility.PIDController;
import org.firstinspires.ftc.teamcode.Utility.PIDFController;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;
import java.util.logging.Logger;

@TeleOp(name="TeleOp", group="TeleOp")
public class MainTeleOp extends OpMode {

    HardwareMapsies robot = new HardwareMapsies();
    MainTeleOpConfig config = new MainTeleOpConfig();

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    List<LynxModule> allHubs;

    Logger dataLogger;

    double holdServoSign = 1;
    double fValue = 0.8;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    double target = 0;

    boolean manualRotation;

    boolean resettingOrientation;

    double launchingPowerOffset = 0;
    private VoltageSensor voltageSensor;
    double frame = 0;

    PIDController rotationPID = new PIDController(1.2, 0, 0.05);
    PIDFController launchingPIDF = new PIDFController(0,0,0, fValue);

    double previousRotation;
    double changeDirection;
    double previousLaunchingPosition;
    ElapsedTime timer;
    double[] prevVelocity = new double[30];
    boolean webcamOn = true;
    boolean firstFrame = true;
    boolean flowerServoUp = true;
    double voltage;

    @Override
    public void init() {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        robot.init(hardwareMap);
        holdServoSign = 1;
        robot.initIMU();

        configurePinpoint();
        previousLaunchingPosition = 0;

        dataLogger = Logger.getLogger("Burg");
        frame = 0;

        initAprilTag();

        CameraCompatibilityManager.getInstance()
                .addQuirk(
                        HardwareMapsies.VENDOR_ID_SUNPLUS_INNOVATION_TECHNOLOGY,
                        HardwareMapsies.PRODUCT_ID_ARDUCAM_OV5648,
                        CameraCompatibilityManager.Quirk.AVOID_LIB_USB_RESET_DEVICE);

        previousRotation = 0;

        changeDirection = 0;
        manualRotation = false;
        resettingOrientation = false;
        launchingPowerOffset = 0;

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        timer = new ElapsedTime();
        fValue = 0.7;
        launchingPIDF.setTarget(0.95);
        webcamOn = true;
        firstFrame = true;
        flowerServoUp = true;

        allHubs = hardwareMap.getAll(LynxModule.class); // See GM0 for information on bulk reads: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // NEVER EVER EVER CALL getPosition ON A MOTOR MORE THAN ONCE
        }
    }

    @Override
    public void start() {
        voltage = voltageSensor.getVoltage();
        fValue = 0.8 - voltage / 12.38 + 1;
        launchingPIDF.setFeedforward(fValue);
    }

    @Override
    public void loop() {

        double launchingMotorPosition = robot.launchingMotor.getCurrentPosition();


        double launchingPower = 0;
        double intakePower = -0.7;

        if(gamepad1.start) {
            resetOrientation();
            resettingOrientation = true;
        } else {
            resettingOrientation = false;
        }

        //TODO: Check latency difference and accuracy between onboard control hub & pinpoint
        robot.pinpoint.update();

        double leftStickY = -gamepad1.left_stick_y;
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.sendTelemetryPacket(packet);


        double leftStickX = -gamepad1.left_stick_x;
        double rightStickX = -gamepad1.right_stick_x;

        double heading = robot.pinpoint.getHeading(AngleUnit.RADIANS);

//        target += rightStickX * 0.05;
        if(Math.abs(rightStickX) > 0){
            manualRotation = true;
            target = heading;
            changeDirection = Math.signum(heading - previousRotation);
        }

        if(Math.abs(rightStickX) == 0 && manualRotation && Math.signum(heading - previousRotation) != changeDirection){
            manualRotation = false;
            target = heading;
        }


        if(gamepad1.dpadUpWasPressed()){
            launchingPowerOffset -= 0.025;
        } else if(gamepad1.dpadDownWasPressed()){
            launchingPowerOffset += 0.025;
        }

        double elapsedTime = timer.seconds();
        timer.reset();

        double deltaEncoder = (robot.launchingMotor.getCurrentPosition() - previousLaunchingPosition);
        double robotVelocity = (deltaEncoder / 8192) / (elapsedTime / 60);

        dashboardTelemetry.addData("deltaEncoder", deltaEncoder);
        dashboardTelemetry.addData("robotVelocity", robotVelocity);

        double tot = 0;

        for(int i = 0; i < prevVelocity.length - 1; i++){
            prevVelocity[i] = prevVelocity[i + 1];
            tot += prevVelocity[i];
        }

        dashboardTelemetry.addData("tot", tot);

        double avg = tot / (prevVelocity.length - 1);
        if(robotVelocity - avg < 725){
            prevVelocity[prevVelocity.length - 1] = robotVelocity;
            avg *= (prevVelocity.length - 1);
            avg += robotVelocity;
            avg /= prevVelocity.length;
        }

        if(gamepad1.xWasPressed()) {
            if(flowerServoUp){
                flowerServoUp = false;
                robot.flowerServo.setPosition(0.37);
            } else {
                flowerServoUp = true;
                robot.flowerServo.setPosition(1);
            }
        }

        dashboardTelemetry.addData("posservoyesmeyes", robot.flowerServo.getPosition());


        if(gamepad1.aWasPressed()) {
            fValue += 0.005;
            launchingPIDF.setFeedforward(fValue);
        }
        if(gamepad1.bWasPressed()) {
            fValue -= 0.005;
            launchingPIDF.setFeedforward(fValue);
        }

        if (gamepad1.right_trigger > 0) {
            frame++;
            if(webcamOn && (firstFrame || frame > 25)){
                frame = 0;
                // Get list of detections from AprilTag Processor
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                // Step through the list of detections and display info for each one.
                for (AprilTagDetection detection : currentDetections) {
                    // Check if the object is an AprilTagSingleDetection or AprilTagClusterDetection
                    if (detection instanceof AprilTagSingleDetection)
                    {
                        //TODO: Make stuff for handling single instances
                    } else {
                        firstFrame = false;
                        AprilTagClusterDetection clusterDetection = (AprilTagClusterDetection) detection;
                        target = Math.toRadians(clusterDetection.ftcPose.bearing) + heading;
                    }
                }
            }
            robot.launchingMotor.setPower(-launchingPIDF.calculate(robot.launchingMotor.getCurrentPosition()));
            robot.holdServo.setPosition(config.getTopServoPos());
        } else {
            robot.launchingMotor.setPower(0);
            robot.holdServo.setPosition(config.getRestingServoPos());
            firstFrame = true;
        }



        rotationPID.setTarget(target);

        double pidPower = resettingOrientation ? 0 : rotationPID.rotationCalculate(heading);

        double lx = leftStickX * Math.cos(heading) - leftStickY * Math.sin(heading);
        double ly = leftStickY * Math.cos(heading) + leftStickX * Math.sin(heading);

        robot.FL.setPower(ly - lx - rightStickX - pidPower);
        robot.FR.setPower(ly + lx + rightStickX + pidPower);
        robot.BL.setPower(ly + lx - rightStickX - pidPower);
        robot.BR.setPower(ly - lx + rightStickX - pidPower);

        dashboardTelemetry.addData("avg", avg);


        if(gamepad1.right_bumper) {
            intakePower = -0.7;
        } else if(gamepad1.left_bumper) {
            intakePower = 0.7;
        }



        robot.intakeMotor.setPower(intakePower);

        previousRotation = heading;
        previousLaunchingPosition = robot.launchingMotor.getCurrentPosition();

        dashboardTelemetry.update();

    }

    public void resetOrientation(){
        robot.pinpoint.resetPosAndIMU();
        target = 0;
    }

    public void configurePinpoint(){ // TODO: Determine the quality difference between pinpoint and built in IMU to save latency
        robot.pinpoint.setOffsets(0, 6, DistanceUnit.INCH);
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        builder.enableLiveView(true);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        visionPortal.setProcessorEnabled(aprilTag, true);

    }
}
