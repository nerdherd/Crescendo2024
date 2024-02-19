package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefDouble implements Preference {
    private double value;
    private String key;

    /**
     * Creates a double preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefDouble(String key, double value) {
        this.key = key;
        this.value = value;
        PreferenceManager.getInstance().addPreference(this);
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.initDouble(key, value);
            value = Preferences.getDouble(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.setDouble(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public double get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(double value) {
        this.value = value;
        if (PreferenceManager.getInstance().isInitialized()) {
            uploadPreferences();
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }
}

