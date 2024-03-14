package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class PrefBool {
    private boolean value;
    private String key;
    private final boolean active;

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
        this(key, value, false);
    }

    public PrefBool(String key, boolean value, boolean isPreference) {
        this.key = key;
        this.value = value;
        this.active = isPreference;
        loadPreferences();
    }

    /**
     * Load preference from robot memory
     */
    public void loadPreferences() {
        if (!active) return;

        Preferences.initBoolean(key, value);
        value = Preferences.getBoolean(key, value);
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (!active) return;

        Preferences.setBoolean(key, value);
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public boolean get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(boolean value) {
        if (!active) return;

        this.value = value;
        uploadPreferences();
    }

}

