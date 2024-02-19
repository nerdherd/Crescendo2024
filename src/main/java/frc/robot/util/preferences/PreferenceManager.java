package frc.robot.util.preferences;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PreferenceManager {
    private static PreferenceManager inst;

    private static final String HARDCODE_MODE_KEY = "USE_DEFAULT_PREFERENCES";
    private static final String CHECK_KEY = "INITIALIZED";
    private static final String CHECK_VALUE = "TRUE";
    private static boolean initialized = false;
    private static boolean useDefaults = false;
    private static Set<Preference> preferences = new HashSet<Preference>();

    public static PreferenceManager getInstance() {
        if (inst == null) {
            inst = new PreferenceManager();
        }
        return inst;
    }

    public void addPreference(Preference pref) {
        if (!preferences.contains(pref)) {
            preferences.add(pref);
        }
    }

    /**
     * Warning: Is a blocking call!
     * 
     * Checks if preferences has initialized, and then waits one second.
     * 
     * If the check fails after 5 minutes, exit.
     */
    public void initialize(boolean force) {
        if (!force) {
            int numTries = 0;
            while (initialized == false && numTries < 300000) {
                initialized = Preferences.getString(CHECK_KEY, "").equals(CHECK_VALUE);
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {}
                numTries++;
            }

            if (!initialized) {
                DriverStation.reportError("Preferences not initialized - timeout after 5 minutes", true);
            }
        }

        useDefaults = Preferences.getBoolean(HARDCODE_MODE_KEY, false);

        // Don't load anything from networktables if we're using defaults
        if (useDefaults) {
            return;
        }

        if (initialized || force) {
            for (Preference pref : preferences) {
                pref.loadPreferences();
            }
        }
    }

    public void initialize() {
        initialize(false);
    }

    public static boolean isInitialized() {
        return initialized;
    }

    public static boolean usingDefaults() {
        return useDefaults;
    }
}
