package frc.robot.util;

public class Conversions {
    public static final double MAG_TICKS = 4096, DEGREES_PER_REVOLUTIONS = 360;

    /**
     * Converts ticks from a Mag Encoder to revolutions.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return revolutions
     */
    public static double magTicksToRevolutions(double magTicks) {
        return magTicks / MAG_TICKS;
    }

    /**
     * Converts revolutions to Mag Encoder ticks.
     *
     * @param revolutions revolutions
     * @return Mag Encoder ticks
     */
    public static double revolutionsToMagTicks(double revolutions) {
        return revolutions * MAG_TICKS;
    }

    /**
     * Converts degrees to revolutions.
     *
     * @param degrees degrees
     * @return revolutions
     */
    public static double degreesToRevolutions(double degrees) {
        return degrees / DEGREES_PER_REVOLUTIONS;
    }

    /**
     * Converts revolutions to degrees.
     *
     * @param revolutions revolutions
     * @return degrees
     */
    public static double revolutionsToDegrees(double revolutions) {
        return revolutions * DEGREES_PER_REVOLUTIONS;
    }

    /**
     * Converts ticks from a Mag Encoder to degrees.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return degrees
     */
    public static double magTicksToDegrees(double magTicks) {
        return revolutionsToDegrees(magTicksToRevolutions(magTicks));
    }

    /**
     * Converts degrees to Mag Encoder ticks.
     *
     * @param degrees degrees
     * @return Mag Encoder ticks
     */
    public static double degreesToMagTicks(double degrees) {
        return revolutionsToMagTicks(degreesToRevolutions(degrees));
    }

    /**
     * Converts motor data to system data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param motorData the motor data
     * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                  rotation.
     * @return the system data
     */
    public static double motorToSystem(double motorData, double gearRatio) {
        return motorData / gearRatio;
    }

    /**
     * Converts system data to motor data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param systemData the system data
     * @param gearRatio  the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                   rotation.
     * @return the motor data
     */
    public static double systemToMotor(double systemData, double gearRatio) {
        return systemData * gearRatio;
    }
}