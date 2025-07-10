// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

import edu.wpi.first.math.Pair;
import yams.motorcontrollers.SmartMotorControllerConfig;

/** Add your docs here. */
public class MotorSetupJson {
    public static class FollowerMotorJson {
        public int canId;
        public boolean inverted = false;
    }

    public String name;
    public String logLevel = "LOW";
    public String motorType;
    public String controllerType;
    public int canId;
    public FollowerMotorJson[] followers = new FollowerMotorJson[0];
    public String idleMode = "BRAKE";
    public UnitValueJson currentLimit = new UnitValueJson(40, UnitsParser.AMPS);
    public boolean inverted = false;
    public int numberOfMotors = 1;

    public static void setupFollowers(SmartMotorControllerConfig motorConfig, MotorSetupJson motorSetup) {
        if (motorSetup.followers != null && motorSetup.followers.length > 0) {
            motorSetup.numberOfMotors += motorSetup.followers.length;
            @SuppressWarnings("unchecked")
            Pair<Object, Boolean>[] followers = new Pair[motorSetup.followers.length];
            for (int i = 0; i < motorSetup.followers.length; i++) {
                Object followerMotor = DeviceConfigReader.getMotor(motorSetup.controllerType, motorSetup.motorType,
                        motorSetup.followers[i].canId);
                followers[i] = new Pair<Object, Boolean>(followerMotor, motorSetup.followers[i].inverted);
            }
            motorConfig.withFollowers(followers);
        }
    }

}
