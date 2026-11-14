package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;

@Config
public class MainTeleOpConfig {

    public static double TOP_SERVO_POS = 1;
    public static double RESTING_SERVO_POS = 0.72;
    public static double LAUNCHING_MOTOR_POWER = -0.8;

    //    public static Double ANGLE_OFFSET = 0.5;
    public double getTopServoPos(){ return TOP_SERVO_POS; }
    public double getRestingServoPos(){ return RESTING_SERVO_POS; }
    public double getLaunchingMotorPower() { return LAUNCHING_MOTOR_POWER; }

//    public Double getAngleOffset() { return ANGLE_OFFSET; }
}
