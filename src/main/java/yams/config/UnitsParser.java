// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.FeetPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Ounces;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * A class that converts a magnitude and a unit into a {@link Measurement}
 * object.
 */
public class UnitsParser {
    public final static String M = "m";
    public final static String CM = "cm";
    public final static String MM = "mm";
    public final static String IN = "in";
    public final static String FT = "ft";
    public final static String YD = "yd";
    public final static String MPS = "m/s";
    public final static String CMPS = "cm/s";
    public final static String MPS2 = "m/s^2";
    public final static String CMPS2 = "cm/s^2";
    public final static String DEG = "deg";
    public final static String RAD = "rad";
    public final static String DEGPS = "deg/s";
    public final static String RADPS = "rad/s";
    public final static String DEGPS2 = "deg/s^2";
    public final static String RADPS2 = "rad/s^2";
    public final static String AMPS = "amps";
    public final static String VOLTS = "volts";
    public final static String SEC = "sec";
    public final static String MS = "ms";
    public final static String US = "us";
    public final static String NS = "ns";
    public final static String KG = "kg";
    public final static String G = "g";
    public final static String MG = "mg";
    public final static String OZ = "oz";
    public final static String LBS = "lbs";
    public final static String STONE = "stone";
    public final static String TONS = "tons";
    private static final String FPS = "ft/s";
    private static final String FPS2 = "ft/s^2";

    /**
     * Converts a magnitude and a unit into a {@link Distance} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      distance.
     * @return The {@link Distance} object.
     */
    public static Distance parseDistance(UnitValueJson unitValueJson) {
        return parseDistance(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link Distance} object.
     * 
     * @param magnitude The magnitude of the distance.
     * @param unit      The unit of the distance. Can be "meters", "feet", "inches",
     *                  "mm", "cm", or "yd".
     *                  (case insensitive)
     * @return The {@link Distance} object.
     */
    public static Distance parseDistance(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case M:
            case "meter":
            case "meters":
                return Meters.of(magnitude);
            case IN:
            case "inch":
            case "inches":
                return Inches.of(magnitude);
            case FT:
            case "foot":
            case "feet":
                return Feet.of(magnitude);
            case "mm":
            case "millimeter":
            case "millimeters":
                return Millimeters.of(magnitude);
            case CM:
            case "centimeter":
            case "centimeters":
                return Centimeters.of(magnitude);
            case YD:
            case "yard":
            case "yards":
                return Feet.of(magnitude * 3);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to bananas");
                return Meters.of(magnitude * (0.254 - Math.random() * 0.05));
        }
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearVelocity} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      velocity.
     * @return The {@link LinearVelocity} object.
     */
    public static LinearVelocity parseVelocity(UnitValueJson unitValueJson) {
        return parseVelocity(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearVelocity} object.
     * 
     * @param magnitude The magnitude of the velocity.
     * @param unit      The unit of the velocity. Can be "meters/second",
     *                  "inches/second",
     *                  "feet/second", "millimeters/second", or
     *                  "centimeters/second".
     *                  (case insensitive)
     * @return The {@link LinearVelocity} object.
     */
    public static LinearVelocity parseVelocity(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case MPS:
            case "m/sec":
            case "meter/sec":
            case "meters/sec":
            case "meter/second":
            case "meters/second":
                return MetersPerSecond.of(magnitude);
            case "in/s":
            case "in/sec":
            case "inch/sec":
            case "inches/sec":
            case "inch/second":
            case "inches/second":
                return InchesPerSecond.of(magnitude);
            case FPS:
            case "ft/sec":
            case "foot/sec":
            case "feet/sec":
            case "foot/second":
            case "feet/second":
                return FeetPerSecond.of(magnitude);
            case "mm/s":
            case "mm/sec":
            case "millimeter/sec":
            case "millimeters/sec":
            case "millimeter/second":
            case "millimeters/second":
                return MetersPerSecond.of(magnitude * 1000);
            case "cm/s":
            case "cm/sec":
            case "centimeter/sec":
            case "centimeters/sec":
            case "centimeter/second":
            case "centimeters/second":
                return MetersPerSecond.of(magnitude * 100);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to meters/second.");
                break;
        }
        return MetersPerSecond.of(magnitude);
    }

    /**
     * Converts a {@link UnitValueJson} object into a {@link LinearAcceleration}
     * object.
     * 
     * @param unitValueJson The {@link UnitValueJson} object containing the
     *                      magnitude
     *                      and unit of the acceleration.
     * @return The {@link LinearAcceleration} object.
     */
    public static LinearAcceleration parseAccelleration(UnitValueJson unitValueJson) {
        return parseAccelleration(unitValueJson.val, unitValueJson.uom);
    }

    /**
     * Converts a magnitude and a unit into a {@link LinearAcceleration} object.
     * 
     * @param magnitude The magnitude of the acceleration.
     * @param unit      The unit of the acceleration. Can be "m/s/s", "m/s2",
     *                  "m/s^2", "m/sec/sec", "m/sec2", "m/sec^2", "meters/sec/sec",
     *                  "meters/sec2", "meters/sec^2", "meters/second/second",
     *                  "meters/second2", "meters/second^2", "in/s/s", "in/s2",
     *                  "in/s^2", "in/sec/sec", "in/sec2", "in/sec^2",
     *                  "inches/sec/sec",
     *                  "inches/sec2", "inches/sec^2", "inches/second/second",
     *                  "inches/second2", "inches/second^2", "ft/s/s", "ft/s2",
     *                  "ft/s^2", "ft/sec/sec", "ft/sec2", "ft/sec^2",
     *                  "feet/sec/sec",
     *                  "feet/sec2", "feet/sec^2", "feet/second/second",
     *                  "feet/second2",
     *                  "feet/second^2", "mm/s/s", "mm/s2", "mm/s^2",
     *                  "millimeter/sec/sec",
     *                  "millimeter/sec2", "millimeter/sec^2",
     *                  "millimeters/sec/sec",
     *                  "millimeters/sec2", "millimeters/sec^2",
     *                  "millimeters/second/second",
     *                  "millimeters/second2", "millimeters/second^2", "cm/s^2",
     *                  "cm/sec^2", "cm/second^2", "centimeter/sec^2",
     *                  "centimeter/second^2",
     *                  "centimeters/sec^2", "centimeters/second^2", "cms/s^2",
     *                  "cms/sec^2", "cms/second^2", "cm/s/s", "cm/sec/sec",
     *                  "cm/second/second", "centimeters/sec/sec",
     *                  "centimeters/second/second".
     *                  (case insensitive)
     * @return The {@link LinearAcceleration} object.
     */
    public static LinearAcceleration parseAccelleration(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case MPS2:
            case "m/s/s":
            case "m/s2":
            case "m/sec/sec":
            case "m/sec2":
            case "m/sec^2":
            case "meters/sec/sec":
            case "meters/sec2":
            case "meters/sec^2":
            case "meters/second/second":
            case "meters/second2":
            case "meters/second^2":
                return MetersPerSecondPerSecond.of(magnitude);
            case "in/s^2":
            case "in/s/s":
            case "in/s2":
            case "in/sec/sec":
            case "in/sec2":
            case "in/sec^2":
            case "inches/sec/sec":
            case "inches/sec2":
            case "inches/sec^2":
            case "inches/second/second":
            case "inches/second2":
            case "inches/second^2":
                return InchesPerSecond.of(magnitude).per(Second);
            case FPS2:
            case "ft/s/s":
            case "ft/s2":
            case "ft/sec/sec":
            case "ft/sec2":
            case "ft/sec^2":
            case "feet/sec/sec":
            case "feet/sec2":
            case "feet/sec^2":
            case "feet/second/second":
            case "feet/second2":
            case "feet/second^2":
                return FeetPerSecondPerSecond.of(magnitude);
            case "mm/s^2":
            case "mm/s/s":
            case "mm/s2":
            case "millimeter/sec/sec":
            case "millimeter/sec2":
            case "millimeter/sec^2":
            case "millimeters/sec/sec":
            case "millimeters/sec2":
            case "millimeters/sec^2":
            case "millimeters/second/second":
            case "millimeters/second2":
            case "millimeters/second^2":
                return MetersPerSecondPerSecond.of(magnitude * 1000);
            case "cm/s^2":
            case "cm/sec^2":
            case "cm/second^2":
            case "centimeter/sec^2":
            case "centimeter/second^2":
            case "centimeters/sec^2":
            case "centimeters/second^2":
            case "cms/s^2":
            case "cms/sec^2":
            case "cms/second^2":
            case "cm/s/s":
            case "cm/sec/sec":
            case "cm/second/second":
            case "centimeters/sec/sec":
            case "centimeters/second/second":
                return MetersPerSecondPerSecond.of(magnitude * 100);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to meters/second.");
                break;
        }
        return MetersPerSecondPerSecond.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Current} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      current.
     * @return The {@link Current} object.
     */
    public static Current parseAmps(UnitValueJson unitValueJson) {
        return parseAmps(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Current} object.
     * 
     * @param magnitude The magnitude of the current.
     * @param unit      The unit of the current. Can be "a", "amp", "amps",
     *                  "ampere", "amperes", "ma", "milliamp",
     *                  "milliamps", "milliampere", "milliamperes", "ua",
     *                  "microamp", "microamps", "microampere", or
     *                  "microamperes".
     *                  (case insensitive)
     * @return The {@link Current} object.
     */
    public static Current parseAmps(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case AMPS:
            case "a":
            case "amp":
            case "ampere":
            case "amperes":
                return Amps.of(magnitude);
            case "ma":
            case "milliamp":
            case "milliamps":
            case "milliampere":
            case "milliamperes":
                return Amps.of(magnitude * 0.001);
            case "ua":
            case "microamp":
            case "microamps":
            case "microampere":
            case "microamperes":
                return Amps.of(magnitude * 0.000001);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to amps.");
                break;
        }
        return Amps.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Voltage} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      voltage.
     * @return The {@link Voltage} object.
     */
    public static Voltage parseVolts(UnitValueJson unitValueJson) {
        return parseVolts(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Voltage} object.
     * 
     * @param magnitude The magnitude of the voltage.
     * @param unit      The unit of the voltage. Can be "v", "volt", "volts",
     *                  "voltage",
     *                  "mv", "millivolt", "millivolts", "uv", "microvolt",
     *                  "microvolts",
     *                  "kv", "kilovolt", or "kilovolts".
     *                  (case insensitive)
     * @return The {@link Voltage} object.
     */
    public static Voltage parseVolts(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case VOLTS:
            case "v":
            case "volt":
            case "voltage":
                return Volts.of(magnitude);
            case "mv":
            case "millivolt":
            case "millivolts":
                return Volts.of(magnitude * 0.001);
            case "uv":
            case "microvolt":
            case "microvolts":
                return Volts.of(magnitude * 0.000001);
            case "kv":
            case "kilovolt":
            case "kilovolts":
                return Volts.of(magnitude * 1000);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to volts.");
                break;
        }
        return Volts.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Time} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      time.
     * @return The {@link Time} object.
     */
    public static Time parseTime(UnitValueJson unitValueJson) {
        return parseTime(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Time} object.
     * 
     * @param magnitude The magnitude of the time.
     * @param unit      The unit of the time. Can be "s", "sec", "second",
     *                  "seconds",
     *                  "ms", "millisecond", "milliseconds", "us", "microsecond",
     *                  "microseconds", "ns", "nanosecond", "nanoseconds", "min",
     *                  "minute", "minutes", "h", "hour", "hours", "d", "day",
     *                  "days".
     *                  (case insensitive)
     * @return The {@link Time} object.
     */
    public static Time parseTime(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case "s":
            case SEC:
            case "second":
            case "seconds":
                return Seconds.of(magnitude);
            case MS:
            case "millisecond":
            case "milliseconds":
                return Seconds.of(magnitude * 0.001);
            case "us":
            case "microsecond":
            case "microseconds":
                return Seconds.of(magnitude * 0.000001);
            case NS:
            case "nanosecond":
            case "nanoseconds":
                return Seconds.of(magnitude * 0.000000001);
            case "min":
            case "minute":
            case "minutes":
                return Seconds.of(magnitude * 60);
            case "h":
            case "hour":
            case "hours":
                return Seconds.of(magnitude * 3600);
            case "d":
            case "day":
            case "days":
                return Seconds.of(magnitude * 86400);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to seconds.");
                break;
        }
        return Seconds.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Mass} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      weight.
     * @return The {@link Mass} object.
     */
    public static Mass parseMass(UnitValueJson unitValueJson) {
        return parseMass(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Mass} object.
     * 
     * @param magnitude The magnitude of the weight.
     * @param unit      The unit of the weight. Can be "kg", "g", "mg", "t",
     *                  "oz", "lb", "st", "pound", "pounds", "kilograms",
     *                  "grams", "milligrams", "ton", "tons", "stone", or
     *                  "stones".
     *                  (case insensitive)
     * @return The {@link Mass} object.
     */
    public static Mass parseMass(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case KG:
            case "kgs":
            case "kilogram":
            case "kilograms":
                return Kilograms.of(magnitude);
            case G:
            case "gram":
            case "grams":
                return Kilograms.of(magnitude * 0.001);
            case MG:
            case "milligram":
            case "milligrams":
                return Kilograms.of(magnitude * 0.000001);
            case TONS:
            case "t":
            case "ton":
                return Kilograms.of(magnitude * 1000);
            case OZ:
            case "ounce":
            case "ounces":
                return Ounces.of(magnitude);
            case LBS:
            case "lb":
            case "pound":
            case "pounds":
                return Pounds.of(magnitude);
            case "st":
            case STONE:
            case "stones":
                return Kilograms.of(magnitude * 6.35029);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to kilograms.");
                break;
        }
        return Kilograms.of(magnitude);
    }

    /**
     * Converts a magnitude and a unit into a {@link Angle} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      angle.
     * @return The {@link Angle} object.
     */
    public static Angle parseAngle(UnitValueJson unitValueJson) {
        return parseAngle(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link Angle} object.
     * 
     * @param magnitude The magnitude of the angle.
     * @param unit      The unit of the angle. Can be "deg", "degrees", "rad", or
     *                  "radians".
     *                  (case insensitive)
     * @return The {@link Angle} object.
     */
    public static Angle parseAngle(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case DEG:
            case "degrees":
                return Degrees.of(magnitude);
            case RAD:
            case "radians":
                return Radians.of(magnitude);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to degrees.");
                return Degrees.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularAcceleration} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      angular acceleration.
     * @return The {@link AngularAcceleration} object.
     */
    public static AngularAcceleration parseAngularAcceleration(UnitValueJson unitValueJson) {
        return parseAngularAcceleration(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularVelocity} object.
     * 
     * @param magnitude The magnitude of the angular velocity.
     * @param unit      The unit of the angular velocity. Can be "deg/s", "deg/sec",
     *                  "deg/second", "degrees/s", "degrees/sec", "degrees/second",
     *                  "rad/s", "rad/sec", "rad/second", "rads/s", "rads/sec",
     *                  "rads/second", "radians", "radians/s", "radians/sec",
     *                  "radians/second". (case insensitive)
     * @return The {@link AngularVelocity} object.
     */
    public static AngularVelocity parseAngularVelocity(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case DEGPS:
            case "deg/sec":
            case "deg/second":
            case "degrees/s":
            case "degrees/sec":
            case "degrees/second":
                return DegreesPerSecond.of(magnitude);
            case RADPS:
            case "rad/sec":
            case "rad/second":
            case "rads/s":
            case "rads/sec":
            case "rads/second":
            case "radians":
            case "radians/s":
            case "radians/sec":
            case "radians/second":
                return RadiansPerSecond.of(magnitude);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to degrees/sec.");
                return DegreesPerSecond.of(magnitude);
        }
    }

    /**
     * Converts a magnitude and a unit into an {@link AngularVelocity} object.
     * 
     * @param unitValueJson A json object containing the magnitude and unit of the
     *                      angular velocity.
     * @return The {@link AngularVelocity} object.
     */
    public static AngularVelocity parseAngularVelocity(UnitValueJson unitValueJson) {
        return parseAngularVelocity(unitValueJson.getMagnitude(), unitValueJson.getUnit());
    }

    /**
     * Converts a magnitude and a unit into a {@link AngularAcceleration} object.
     * 
     * @param magnitude The magnitude of the angular acceleration.
     * @param unit      The unit of the angular acceleration. Can be "deg/s/s",
     *                  "deg/s2", "deg/s^2", "degrees/s/s", "degrees/s2",
     *                  "degrees/s^2", "rads/s/s", "rads/s2", "rads/s^2", "radians",
     *                  "radians/s/s", "radians/s2", "radians/s^2".
     *                  (case insensitive)
     * @return The {@link AngularAcceleration} object.
     */
    public static AngularAcceleration parseAngularAcceleration(double magnitude, String unit) {
        switch (unit.trim().toLowerCase()) {
            case DEGPS2:
            case "deg/s/s":
            case "deg/s2":
            case "degrees/s/s":
            case "degrees/s^2":
            case "degrees/s2":
                return DegreesPerSecondPerSecond.of(magnitude);
            case RADPS2:
            case "rads/s/s":
            case "rads/s2":
            case "radians":
            case "radians/s/s":
            case "radians/s^2":
            case "radians/s2":
                return RadiansPerSecondPerSecond.of(magnitude);
            default:
                System.err.println("Unknown unit: " + unit + " for " + magnitude + ". Defaulting to degrees/sec^2.");
                return DegreesPerSecondPerSecond.of(magnitude);
        }
    }
}
