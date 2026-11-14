package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.HardwareMapsies;
import org.firstinspires.ftc.teamcode.Utility.PIDFController;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

@Autonomous(name="Main Auto", group="TeleOp")
public class MainAuto extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55, 9, 90);
    private final Pose shootingPose = poseFactory.of(28, 28, 43);
    private final Pose intake1 = poseFactory.of(60, 19.1, 270);
    private final Pose intake2 = poseFactory.of(60, 14, 270);

//    private final Pose controlPoseIntake1 = poseFactory.of(45, 13, 0);
    private final Pose controlPoseIntake1 = poseFactory.of(60, 60, 0);
    private final Pose finalPosition = poseFactory.of(11, 98.13, 0);

    boolean shot = false;
    boolean timeStart = false;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime timer2 = new ElapsedTime();
    double previousLaunchingPosition = 0;
    double intakePower = 0;
    double servoPosition = 0;
    private VoltageSensor voltageSensor;

    private Path shootingPose(){
        return line(startPose, shootingPose).linear(startPose, shootingPose);
    }
    private Path intake1Path(){
        return curve(shootingPose, controlPoseIntake1, intake1).tangent();
    }

    private Path intake2Path(){
        return curve(shootingPose, controlPoseIntake1, intake2).tangent();
    }

    private Path intakeToShoot(){
        return curve(intake1, controlPoseIntake1, shootingPose).heading(
                Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(intake1))
                        .until(1, Interpolator.linear(intake1, shootingPose))
        );
    }

    private Path shootToFinal(){
        return line(shootingPose, finalPosition).linear(shootingPose, finalPosition);
    }

    private Path intake2ToShoot(){
        return curve(intake2, controlPoseIntake1, shootingPose).heading(
                Interpolator.piecewise()
                        .until(0.3, Interpolator.constant(intake1))
                        .until(1, Interpolator.linear(intake2, shootingPose))
        );
    }

    private Command shootingRoutine = Command.build()
            .setStart(() -> shot = false)
            .setExecute(this::shoot)
            .setDone(() -> shot)
            .setEnd(end -> {
                launchingPower = 0;
                intakePower = 0;
                servoPosition = 0.72;
                timeStart = false;
                shot = false;
                prevVelocity = new double[30];
            });

//    private Command shootingRoutine2 = Command.build()
//            .setStart(() -> shot = false)
//            .setExecute(this::shoot)
//            .setDone(() -> shot)
//            .setEnd(end -> {
//                launchingPower = 0;
//                intakePower = 0;
//                servoPosition = 0.72;
//                timeStart = false;
//                shot = false;
//            });


    private Command intakeOn = Command.build()
            .setStart(() -> intakePower = -0.8)
            .setDone(() -> true);

    private Command intakeOff = Command.build()
            .setStart(() -> intakePower = 0)
            .setDone(() -> true);
    private Command autoRoutine(){
        return sequential(
                follow(follower, shootingPose()),
                shootingRoutine,
                intakeOn,
                follow(follower, intake1Path()),
                intakeOff,
                follow(follower, intakeToShoot()),
                shootingRoutine,
                intakeOn,
                follow(follower, intake2Path()),
                follow(follower, intake2ToShoot()),
                shootingRoutine,
                follow(follower, shootToFinal())
        );
    }



    HardwareMapsies robot = new HardwareMapsies();
    double fValue = 0.8;
    PIDFController launchingPIDF = new PIDFController(0,0,0, fValue);

    double[] prevVelocity = new double[30];
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    double launchingPower = 0;

    @Override
    public void init() {
        Scheduler.reset();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        robot.init(hardwareMap);
        launchingPower = 0;
        servoPosition = 0.72;
        intakePower = 0;
        timeStart = false;

        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
    }
    double voltage;

    @Override
    public void start() {
        voltage = voltageSensor.getVoltage();
        launchingPIDF.setFeedforward(0.8 - voltage / 12.5 + 1);
        schedule(autoRoutine());
    }

    @Override
    public void loop() {

        follower.update();
        Scheduler.execute();

        robot.launchingMotor.setPower(launchingPower);
        robot.intakeMotor.setPower(intakePower);
        robot.holdServo.setPosition(servoPosition);

        dashboardTelemetry.addData("X", follower.pose().x());
        dashboardTelemetry.addData("Y", follower.pose().y());
        dashboardTelemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        dashboardTelemetry.addData("State", follower.mode());
        dashboardTelemetry.addData("", follower.closestPose());
        dashboardTelemetry.addData("Completion", follower.completion());
        dashboardTelemetry.addData("voltage", voltage);

        dashboardTelemetry.addData("state", Scheduler.isScheduled(shootingRoutine));

        dashboardTelemetry.update();

    }

    public void shoot(){
        double elapsedTime = timer.seconds();
        timer.reset();
        intakePower = 0;
        double deltaEncoder = (robot.launchingMotor.getCurrentPosition() - previousLaunchingPosition);
        double robotVelocity = (deltaEncoder / 8192) / (elapsedTime / 60);

        dashboardTelemetry.addData("deltaEncoder", deltaEncoder);
        dashboardTelemetry.addData("robotVelocity", robotVelocity);

        double tot = 0;

        for(int i = 0; i < prevVelocity.length - 1; i++){
            prevVelocity[i] = prevVelocity[i + 1];
            tot += prevVelocity[i];
        }
        servoPosition = 1;

        dashboardTelemetry.addData("tot", tot);

        double avg = tot / (prevVelocity.length - 1);
        if(robotVelocity - avg < 850){
            prevVelocity[prevVelocity.length - 1] = robotVelocity;
            avg *= (prevVelocity.length - 1);
            avg += robotVelocity;
            avg /= prevVelocity.length;
        }

        dashboardTelemetry.addData("Avg",  avg);

        double deviation = 0;
        for(int i = prevVelocity.length - 10; i < prevVelocity.length; i++){
            deviation += prevVelocity[i] - prevVelocity[i - 1];
        }

        if(Math.abs(avg - 2425) < 110){
            if(!timeStart){
                timer2.reset();
                timeStart = true;
            }
            intakePower = -0.8;
        }
        previousLaunchingPosition = robot.launchingMotor.getCurrentPosition();
        launchingPIDF.setTarget(0.95);
        launchingPower = -launchingPIDF.calculate(0);
        dashboardTelemetry.addData("deviation", deviation);
        if(deviation < -600 || (timeStart && timer2.seconds() > 4)) {
            shot = true;
        }
    }
}
