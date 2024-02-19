package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefFloat implements Preference {
    private float value;
    private final float defaultValue;
    private String key;

    /**
     * Create a float preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefFloat(String key, float value) {
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
            Preferences.initFloat(key, value);
            value = Preferences.getFloat(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.isInitialized()) {
            Preferences.setFloat(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public float get() {
        if (PreferenceManager.usingDefaults()) {
            return defaultValue;
        }

        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(float value) {
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

