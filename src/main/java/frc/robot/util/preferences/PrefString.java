package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefString implements Preference {
    private String value;
    private String key;

    /**
     * Create a string preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefString(String key, String value) {
        this.key = key;
        this.value = value;
        PreferenceManager.getInstance().addPreference(this);
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.initString(key, value);
            value = Preferences.getString(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.setString(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public String get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(String value) {
        this.value = value;
        if (PreferenceManager.getInstance().isInitialized()) {
            uploadPreferences();
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

}

