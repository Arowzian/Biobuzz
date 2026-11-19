package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utility.HardwareMapsies;
import org.firstinspires.ftc.teamcode.Utility.PIDFController;

import java.util.List;

@TeleOp(name="PID Tuning", group="Tuning")
public class PIDTuning extends OpMode {

    // Hardware Map
    HardwareMapsies robot = new HardwareMapsies();

    // FTC Dashboard
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    // Bulk Reads
    List<LynxModule> allHubs;

    // Voltage Sensor
    private VoltageSensor voltageSensor;
    double voltage;

    double[] values = {0, 0, 0, 0.245};

    // PIDF
    PIDFController launchingPIDF = new PIDFController(values[0],values[1],values[2], values[3]);

    // Data Collection
    double[] prevVelocity = new double[30];
    double previousLaunchingPosition;
    ElapsedTime timer;

    @Override
    public void init() {
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        robot.init(hardwareMap);
        robot.initIMU();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        timer = new ElapsedTime();

        launchingPIDF.setTarget(2.75);

        robot.holdServo.setPosition(1); // Set to open

        allHubs = hardwareMap.getAll(LynxModule.class); // See GM0 for information on bulk reads: https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO); // NEVER EVER EVER CALL getPosition ON A MOTOR MORE THAN ONCE
        }
    }

    @Override
    public void start() {
        voltage = voltageSensor.getVoltage();
    }
    String[] selectedTerm = {"P", "I", "D", "F"};
    int selected = 0;
    final double stepSize = 0.0025;
    @Override
    public void loop() {

        int launchingMotorPosition = robot.launchingMotor.getCurrentPosition();

        if(gamepad1.right_bumper){ robot.intakeMotor.setPower(-0.8); }
        else { robot.intakeMotor.setPower(0); }

        double elapsedTime = timer.seconds(); // Get elapsed time
        timer.reset(); // Resets the timer back to 0 for use in the next tick

        double deltaEncoder = (launchingMotorPosition - previousLaunchingPosition);

        double motorVelocity = (deltaEncoder / 8192) / (elapsedTime / 60); // RPM

        double power = -launchingPIDF.calculate(motorVelocity / 1000); // Note that the thousand is simply for floating point errors

        robot.launchingMotor.setPower(power);

        // Summation of previous RPMs
        double tot = 0;

        // Shift each velocity in the array over 1, and add it to the total
        for(int i = 0; i < prevVelocity.length - 1; i++){
            prevVelocity[i] = prevVelocity[i + 1];
            tot += prevVelocity[i];
        }

        double avg = tot / (prevVelocity.length - 1);

        // See if the current Robot Velocity is within 725 RPM of the average to remove outliers (probably caused by latency spikes)
        if(motorVelocity - avg < 725){
            // If we can determine that the change isn't too extreme, we add it to the total and redo the average
            prevVelocity[prevVelocity.length - 1] = motorVelocity;
            avg *= (prevVelocity.length - 1);
            avg += motorVelocity;
            avg /= prevVelocity.length;
        }

        if(gamepad1.dpadUpWasPressed()) {
            if(selected < 3){
                selected++;
            } else {
                selected = 0;
            }
        }

        if(gamepad1.aWasPressed()){
            values[selected] = values[selected] + stepSize;
        }

        if(gamepad1.bWasPressed()){
            values[selected] = values[selected] - stepSize;
        }

        launchingPIDF.setPCoefficent(values[0]);
        launchingPIDF.setICoefficent(values[1]);
        launchingPIDF.setDCoefficent(values[2]);
        launchingPIDF.setFeedforward(values[3]);

        dashboardTelemetry.addData("Average", avg);
        dashboardTelemetry.addData("Voltage", voltage);
        dashboardTelemetry.addData("Error", launchingPIDF.getError());
        dashboardTelemetry.addData(selectedTerm[selected], values[selected]);

        previousLaunchingPosition = launchingMotorPosition;

        dashboardTelemetry.update();
    }
}
