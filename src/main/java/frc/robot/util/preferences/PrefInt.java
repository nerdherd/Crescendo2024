package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PrefInt implements Preference {
    private int value;
    private String key;

    /**
     * Create a int preference with the provided key and value
     * <p>
     * Loads the preference into robot memory. 
     * If the robot memory already contains a value with the same key, 
     * it will overwrite the provided value.
     * 
     * @param key
     * @param value
     */
    public PrefInt(String key, int value) {
        this.key = key;
        this.value = value;
        PreferenceManager.getInstance().addPreference(this);
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.initInt(key, value);
            value = Preferences.getInt(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (PreferenceManager.getInstance().isInitialized()) {
            Preferences.setInt(key, value);
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public int get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(int value) {
        this.value = value;
        if (PreferenceManager.getInstance().isInitialized()) {
            uploadPreferences();
        } else {
            DriverStation.reportError("Preferences not initialized!", true);
        }
    }

}

