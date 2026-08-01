package frc.demacia.utils.log;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.util.sendable.Sendable;

public class Dashboard {
    private static final Map<String, DashboardBuilder> builders = new HashMap<>();

    public static void putData(String key, Sendable sendable) {
        DashboardBuilder builder = builders.computeIfAbsent(key, DashboardBuilder::new);
        if (sendable instanceof NTSendable ntSendable) {
            ntSendable.initSendable(builder);
        } else {
            sendable.initSendable(builder);
        }
    }

    public static void putNumber(String key, double value) {
        Log.putData(key, () -> value);
    }

    public static void putBoolean(String key, boolean value) {
        Log.putData(key, () -> value);
    }

    public static void putString(String key, String value) {
        Log.putData(key, () -> value);
    }

    public static void periodic() {
        for (DashboardBuilder builder : builders.values()) {
            builder.pollInputs();
            builder.update();
        }
    }
}