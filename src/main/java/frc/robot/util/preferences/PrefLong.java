package frc.robot.util.preferences;

import edu.wpi.first.wpilibj.Preferences;

public class PrefLong {
    private long value;
    private String key;
    private final boolean active;

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
        this(key, value, false);
    }

    public PrefLong(String key, long value, boolean isPreference) {
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

        Preferences.initLong(key, value);
        value = Preferences.getLong(key, value);
    }

    /**
     * Upload the current value of the preference in code to the robot memory
     */
    public void uploadPreferences() {
        if (!active) return;

        Preferences.setLong(key, value);
    }

    /**
     * Get the current value of this preference.
     * @return value
     */
    public long get() {
        return this.value;
    }

    /**
     * Set the current value of this preference.
     * @param value
     */
    public void set(long value) {
        if (!active) return;

        this.value = value;
        uploadPreferences();
    }

}

