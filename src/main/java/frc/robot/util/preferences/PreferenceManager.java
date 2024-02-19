package frc.robot.util.preferences;

import java.util.HashSet;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;

public class PreferenceManager {
    private static PreferenceManager inst;

    private static final String DEFAULT_MODE_KEY = "USE_DEFAULT_PREFERENCES";
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
     * If the check fails after 1 minute, exit.
     */
    public void initialize(boolean forceLoad) {
        if (!forceLoad) {
            int numTries = 0;

            // Wait until initialized
            while (!initialized && numTries < 60000) {
                DriverStation.reportWarning("Preferences Initializing... " + numTries + " seconds elapsed ", true);
                // Check if initialized
                initialized = Preferences.getString(CHECK_KEY, "").equals(CHECK_VALUE);

                // Sleep for 1 second
                try {
                    Thread.sleep(1000);
                } catch (Exception e) {}
                numTries++;
            }

            if (!initialized) {
                DriverStation.reportError("Preferences not initialized - timeout after 1 minute", true);
            }
        }
        
        // Check if default mode is on
        Preferences.initBoolean(DEFAULT_MODE_KEY, false);
        useDefaults = Preferences.getBoolean(DEFAULT_MODE_KEY, false);

        // Don't load anything from networktables if we're using defaults
        if (useDefaults) {
            return;
        }

        // Load every preference
        if (initialized || forceLoad) {
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
