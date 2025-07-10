package yams.config;

import java.io.File;
import java.util.Optional;

import edu.wpi.first.math.system.plant.DCMotor;
import yams.config.ReflectionsManager.VENDOR;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorFactory;
import yams.motorcontrollers.local.SparkWrapper.SparkBaseType;

/** A class for reading device configurations from JSON files. */
public class DeviceConfigReader {

  /**
   * Get a {@link SmartMotorController} wrapper from the provided motor controller object and simulate
   * it with the provided motor simulator.
   *
   * @param controller Motor controller type. Supported types are "spark", "sparkmax", "talonfx",
   *     "talonfxs", "nova", "thrifty", "thrifty_nova", and "thriftynova".
   * @param type Motor type. Supported types are "neo", "neo550", "krakenx60", and "krakenx60foc".
   * @param id Motor ID.
   * @param config {@link SmartMotorControllerConfig} for the motor controller.
   * @return {@link SmartMotorController} wrapper for the motor controller.
   */
  public static Optional<SmartMotorController> getSmartMotor(String controller, String type, int id, SmartMotorControllerConfig config) {
    Optional<SmartMotorController> motor = Optional.empty();
    DCMotor motorSim = null;
    int numberOfMotors = config.getFollowers().map(it -> it.length + 1).orElse(1);
    Object motorController = getMotor(controller, type, id);
    switch (type.toLowerCase()) {
      case "neo":
        motorSim = DCMotor.getNEO(numberOfMotors);
        break;
      case "neo550":
        motorSim = DCMotor.getNeo550(numberOfMotors);
        break;  
      case "krakenx60":
        motorSim = DCMotor.getKrakenX60(numberOfMotors);
        break;
      case "krakenx60foc":
        motorSim = DCMotor.getKrakenX60Foc(numberOfMotors);
        break;  
      default:
    }

    motor = SmartMotorFactory.create(((SmartMotorController)motorController).getMotorController(), motorSim, config);
    return motor;
  }

  /**
   * Get a motor controller based on the controller type and motor type.
   *
   * @param controller The type of motor controller, e.g. "spark", "talonfx", "nova", "thrifty", "thrifty_nova", or "thriftynova"
   * @param type The type of motor, e.g. "neo", "neo550", "krakenx60", or "krakenx44"
   * @param id The CAN ID of the motor controller
   * @return The motor controller object
   */
  public static Object getMotor(String controller, String type, int id) {
    ReflectionsManager.VENDOR vendorType =VENDOR.REV;
    switch (controller.toLowerCase()) {
      case "spark":
      case "sparkmax":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.SparkWrapper", 
          new Class[]{int.class, SparkBaseType.class}, new Object[]{id, SparkBaseType.SPARK_MAX});
      case "sparkflex":
        vendorType = VENDOR.REV;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.SparkWrapper", 
          new Class[]{int.class, SparkBaseType.class}, new Object[]{id, SparkBaseType.SPARK_FLEX});
      case "talonfx":
        vendorType = VENDOR.PHOENIX6;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.remote.TalonFXWrapper", 
          new Class[]{int.class}, new Object[]{id});  
      case "talonfxs":
      case "nova":
        vendorType = VENDOR.PHOENIX6;
        return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.remote.TalonFXSWrapper", 
          new Class[]{int.class}, new Object[]{id});  
      case "thrifty":
      case "thrifty_nova":
      case "thriftynova":
          vendorType = VENDOR.THRIFTYBOT;
          return ReflectionsManager.<SmartMotorController>create(vendorType, "yams.motorcontrollers.local.NovaWrapper", 
            new Class[]{int.class}, new Object[]{id});  
      default:
        throw new RuntimeException("Unknown motor controller type: " + controller);
    }
  }

  /**
   * Method to check the existence of specific JSON configuration files in the provided directory.
   *
   * @param directory the directory to check for JSON configuration files
   */
  public static void checkDirectory(File directory) {
    assert new File(directory, "robot.json").exists();
  }
}
