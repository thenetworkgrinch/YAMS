// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

/** Add your docs here. */
public class MotorSystemIdJson {
    public static class FeedBack {
        public double p = 0;
        public double i = 0;
        public double d = 0;
    }
    public static class FeedForward {
        public double s = 0;
        public double g = 0;
        public double v = 0;
        public double a = 0;
    }
    public UnitValueJson closedLoopRamp = new UnitValueJson(0.25, UnitsParser.SEC);
    public UnitValueJson openLoopRamp = new UnitValueJson(0.25, UnitsParser.SEC);
    public FeedBack feedBack;
    public UnitValueJson maxVelocity = new UnitValueJson(0, UnitsParser.MPS);
    public UnitValueJson maxAcceleration = new UnitValueJson(0, UnitsParser.MPS2);
    public FeedForward feedForward = new FeedForward();
    public String controlMode = "CLOSED_LOOP";
}
