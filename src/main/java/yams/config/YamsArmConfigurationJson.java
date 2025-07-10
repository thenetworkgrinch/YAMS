// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.config;

import java.util.Optional;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

public class YamsArmConfigurationJson implements DeviceConfiguration {
    public MotorSetupJson motorSetup = new MotorSetupJson();
    public MotorSystemIdJson motorSystemId = new MotorSystemIdJson();
    public UnitValueJson length = new UnitValueJson(0, UnitsParser.IN);
    public UnitValueJson lowerHardLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson upperHardLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson startingAngle = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson lowerSoftLimit = new UnitValueJson(0, UnitsParser.DEG);
    public UnitValueJson upperSoftLimit = new UnitValueJson(0, UnitsParser.DEG);
    public double[] gearing;
    public UnitValueJson mass = new UnitValueJson(0, UnitsParser.LBS);
    public UnitValueJson voltageCompensation = new UnitValueJson(12, UnitsParser.VOLTS);
    public UnitValueJson horizontalZero = new UnitValueJson(0, UnitsParser.DEG);

    @Override
    public Arm configure(SubsystemBase deviceHandler) {
        SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(deviceHandler)
                .withClosedLoopController(motorSystemId.feedBack.p, motorSystemId.feedBack.i, motorSystemId.feedBack.d,
                        UnitsParser.parseAngularVelocity(motorSystemId.maxVelocity),
                        UnitsParser.parseAngularAcceleration(motorSystemId.maxAcceleration))
                .withSoftLimit(UnitsParser.parseAngle(lowerSoftLimit),
                        UnitsParser.parseAngle(upperSoftLimit))
                .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(gearing)))
                .withIdleMode(MotorMode.valueOf(motorSetup.idleMode))
                .withTelemetry(motorSetup.name + "Motor", TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withStatorCurrentLimit(UnitsParser.parseAmps(motorSetup.currentLimit))
                .withMotorInverted(motorSetup.inverted)
                .withClosedLoopRampRate(UnitsParser.parseTime(motorSystemId.closedLoopRamp))
                .withOpenLoopRampRate(UnitsParser.parseTime(motorSystemId.openLoopRamp))
                .withFeedforward(new ArmFeedforward(motorSystemId.feedForward.s, motorSystemId.feedForward.g,
                        motorSystemId.feedForward.v, motorSystemId.feedForward.a))
                .withControlMode(ControlMode.valueOf(motorSystemId.controlMode));
        MotorSetupJson.setupFollowers(motorConfig, motorSetup);

        Optional<SmartMotorController> smartMotor = DeviceConfigReader.getSmartMotor(motorSetup.controllerType, motorSetup.motorType, motorSetup.canId, motorConfig);
        ArmConfig armConfig = new ArmConfig(smartMotor.get())
                .withLength(UnitsParser.parseDistance(length))
                .withHardLimit(UnitsParser.parseAngle(lowerHardLimit),
                        UnitsParser.parseAngle(upperHardLimit))
                .withTelemetry(motorSetup.name, TelemetryVerbosity.valueOf(motorSetup.logLevel))
                .withMass(UnitsParser.parseMass(mass))
                .withStartingPosition(UnitsParser.parseAngle(startingAngle))
                .withHorizontalZero(UnitsParser.parseAngle(horizontalZero));
        Arm arm = new Arm(armConfig);
        return arm;
    }

}