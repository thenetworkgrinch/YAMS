package yams.config;

import java.util.HashMap;
import java.util.Map;

/**
 * Create classes only if the vendor dep exists.
 */
public class ReflectionsManager
{

  /**
   * Vendors that supply their own vendordep to communicate with their products.
   */
  public enum VENDOR
  {
    /**
     * REVLib
     */
    REV,
    /**
     * CTRE Phoenix 5 and 6
     */
    PHOENIX5,
    PHOENIX6,
    /**
     * ThriftyLib
     */
    THRIFTYBOT
  }

  private static final Map<VENDOR, String> vendorLibs = new HashMap<VENDOR, String>() {{
    put(VENDOR.REV, "com.revrobotics.spark.SparkBase");
    put(VENDOR.PHOENIX5, "com.ctre.phoenix.hardware.TalonSRX");
    put(VENDOR.PHOENIX6, "com.ctre.phoenix6.hardware.TalonFX");
    put(VENDOR.THRIFTYBOT, "com.thriftybot.hardware.ThriftyBot");
  }};

  /**
   * Check if the vendordep exists.
   *
   * @param vendor Vendor to check for their library.
   * @return Boolean on existence of their library.
   */
  public static boolean checkIfVendorLibExists(VENDOR vendor)
  {
    try
    {
      // If the class is found, the library exists
      Class.forName(vendorLibs.get(vendor));
    } catch (Exception e)
    {
      return false;
    }
    return true;
  }

  /**
   * Create objects if the vendordep exists. Throw an exception when they dont.
   *
   * @param v              Vendor to check if the vendordep exists.
   * @param className      Wrapper classname to create.
   * @param parameterTypes Parameter types for the wrappers constructor.
   * @param parameters     Parameters for the wrappers constructor
   * @param <T>            Wrapper type.
   * @return Wrapper object.
   */
  @SuppressWarnings("unchecked")
  public static <T> T create(VENDOR v, String className, Class<?>[] parameterTypes, Object[] parameters)
  {
    if (!checkIfVendorLibExists(v))
    {
      throw new RuntimeException("Vendor " + v + " library not found! Please install it!");
    }
    try
    {
      Class<?> wrapper   = Class.forName(className);
      Object   vendorObj = wrapper.getDeclaredConstructor(parameterTypes).newInstance(parameters);
      return (T) vendorObj;
    } catch (Exception e)
    {
      throw new RuntimeException(e);
    }
  }
}