package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefBool implements Preference {
    private boolean value;
    private final boolean defaultValue;
    private String key;

    /**
     * Creates a boolean preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefBool(String key, boolean value) {
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
            Preferences.initBoolean(key, value);
            value = Preferences.getBoolean(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.isInitialized()) {
            Preferences.setBoolean(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public boolean get() {
        if (PreferenceManager.usingDefaults()) {
            return defaultValue;
        }

        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(boolean value) {
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

}

