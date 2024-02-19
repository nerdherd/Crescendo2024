package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefLong implements Preference {
    private long value;
    private final long defaultValue;
    private String key;

    /**
     * Create a long preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefLong(String key, long value) {
        this.key = key;
        this.value = value;
        this.defaultValue = value;
        PreferenceManager.getInstance().addPreference(this);
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        if (PreferenceManager.usingDefaults()) {
            this.value = defaultValue;
            return;
        }

        if (PreferenceManager.isInitialized()) {            
            Preferences.initLong(key, value);
            value = Preferences.getLong(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.isInitialized()) {
            Preferences.setLong(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public long get() {
        if (PreferenceManager.usingDefaults()) {
            return defaultValue;
        }

        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(long value) {
        if (PreferenceManager.usingDefaults()) {
            return;
        }

        this.value = value;
        if (PreferenceManager.isInitialized()) {
            uploadPreferences();
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    public String getKey() {
        return this.key;
    }
}

