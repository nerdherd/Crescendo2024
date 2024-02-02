package frc.robot.util;

public class NerdyMath {
    /**
     * Re-maps a number from one range to another.
     * 
     * Similar implementation to the arduino 
     * <a href="https://reference.arduino.cc/reference/en/language/functions/math/map/">
     * Math.map()</a> method.
     * 
     * <p>
     * 
     * Example: map(0.75, 0, 1, 1, 0) returns 0.25.
     * 
     * <p>
     * 
     * Does not ensure that a number will stay within the range. 
     * Use {@link #clamp() clamp()} to do so.
     * 
     * @see https://github.com/arduino/ArduinoCore-API/blob/master/api/Common.cpp
     * 
     * @return the re-mapped number
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Maps x to the range [0, mod]
     */
    public static double posMod(double x, double mod) {
        return ((x % 1) + 1) % 1;
    }

    public static double degreesToRadians(double deg) {
        return deg * Math.PI/180;
    }

    public static double radiansToDegrees(double rad) {
        return rad * 180 / Math.PI;
    }

    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Checks if the value is within the range (inclusive)
     */
    public static boolean inRange(double myValue, double min, double max) {
        return (myValue >= min) && (myValue <= max);
    }
    
    /**
     * Checks if the value is within the range (noninclusive)
     */
    public static boolean inRangeOpen(double myValue, double min, double max) {
        return (myValue > min) && (myValue < max);
    }

    public static double deadband(double value, double min, double max) {
        if(inRange(value, min, max)) return 0;
        return value;
    }

    public static double standardDeviation(double[] values) {
        double sum = 0.0, standardDeviation = 0.0;
        
        for(int i = 0; i < values.length; i++) {
            sum += values[i];
        }

        double mean = sum / values.length;

        for (int i = 0; i < values.length; i++) {
            standardDeviation += Math.pow(values[i] - mean, 2);
        }

        return Math.sqrt(standardDeviation / values.length);
    }

    public static boolean withinStandardDeviation(double[] values, int stdevsAway, double newValue) {
        double sum = 0.0;
        
        for(int i = 0; i < values.length; i++) {
            sum += values[i];
        }

        double mean = sum / values.length;
        double stdev = standardDeviation(values);

        if(newValue >= mean - stdevsAway*stdev && newValue <= mean + stdevsAway*stdev) return true;
        return false;
    }

}