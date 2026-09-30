// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.log;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.RobotCommon;
import frc.demacia.utils.sysid.SysidCommand;

/**
 * Centralized logging system for robot telemetry and diagnostics.
 * <p>
 * Manages the creation, updating, and optimization of log entries.
 * Handles both file logging (DataLog) and live dashboard updates
 * (NetworkTables).
 * </p>
 */
public class Log extends SubsystemBase {

  /**
   * Enumeration for different logging levels.
   * Defines behavior for file logging and NetworkTables updating, both in and out of competition.
   */
  public static enum LogLevel { 
    /** Log to file only, but remove entirely during competition */
    LOG_ONLY_NOT_IN_COMP, 
    /** Log to file only */
    LOG_ONLY, 
    /** Log to file and NetworkTables only when not in competition */
    LOG_AND_NT_NOT_IN_COMP, 
    /** Log to file and NetworkTables */
    LOG_AND_NT
  } 

  private static final Map<String, DashboardBuilder> builders = new HashMap<>();

  /** Singleton instance of the LogManager */
  private static Log logManager;

  /** The main DataLog instance for file writing */
  public static DataLog log;
  /** The NetworkTable instance for the "Log" table */
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("Log");

  /** List of currently active console alerts */
  private static ArrayList<ConsoleAlert> activeConsole;

  /** List of individual log entries that are not grouped */
  private ArrayList<LogEntry<?>> individualLogEntries = new ArrayList<>();

  private LogEntry<float[]> groupFloatEntry;
  private LogEntry<boolean[]> groupBooleanEntry;
  private LogEntry<String[]> groupStringEntry;

  /**
   * Private constructor to enforce Singleton pattern.
   * Initializes DataLogManager and starts logging.
   */
  private Log() {
    if (logManager != null) {
      CommandScheduler.getInstance().unregisterSubsystem(this);
      return;
    }
    logManager = this;
    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);

    activeConsole = new ArrayList<>();
    log("log manager is ready");

    SmartDashboard.putData("sysID/sysidCommand", new SysidCommand());
    SmartDashboard.putData("replay/LoadLatestLog", new LogReplayCommand());
    RobotCommon.init();
  }

  /**
   * Static initializer to ensure the LogManager is created.
   */
  static {
    if (logManager == null) {
      new Log();
    }
  }

  /**
   * Removes non-essential log entries when in competition mode.
   * Cleans up both individual and categorized entries based on their LogLevel.
   */
  public static void removeInComp() {
    if (logManager == null)
      return;

    for (int i = 0; i < logManager.individualLogEntries.size(); i++) {
      LogEntry<?> entry = logManager.individualLogEntries.get(i);
      entry.removeInComp();
      if (entry.getLogLevel() == LogLevel.LOG_ONLY_NOT_IN_COMP
          || logManager.individualLogEntries.get(i).getLogLevel() == LogLevel.LOG_AND_NT_NOT_IN_COMP) {
        logManager.individualLogEntries.remove(i);
        i--;
      }
    }
  }

  /**
   * Clears all log entries from the manager.
   */
  public static void clearEntries() {
    if (logManager != null) {
      logManager.individualLogEntries.clear();
    }
  }

  /**
   * Logs a message to the console and creates an alert.
   * Manages the console limit by removing old alerts.
   * 
   * @param message   The message to log
   * @param alertType The severity of the alert
   * @return The created ConsoleAlert
   */
  public static ConsoleAlert log(Object message, AlertType alertType) {
    DataLogManager.log(String.valueOf(message));

    ConsoleAlert alert = new ConsoleAlert(String.valueOf(message), alertType);
    alert.set(true);
    if (activeConsole.size() > ConsoleConstants.CONSOLE_LIMIT) {
      activeConsole.get(0).close();
      activeConsole.remove(0);
    }
    activeConsole.add(alert);
    return alert;
  }

  /**
   * Logs an info message to the console.
   * 
   * @param message The message to log
   * @return The created ConsoleAlert
   */
  public static ConsoleAlert log(Object message) {
    return log(message, AlertType.kInfo);
  }

  /**
   * Alerts created by {@link #alert(String, boolean)}, keyed by their message.
   * <p>
   * This map is what makes the one liner API possible: the caller does not hold the alert
   * object, so it is looked up here by its text and reused on every loop. Reusing the same
   * object is exactly what keeps the edge detection inside {@link ConsoleAlert} working - a
   * new object every loop would reset its state and spam the log and the dashboard.
   * </p>
   */
  private static final Map<String, ConsoleAlert> conditionAlerts = new HashMap<>();

  /** Single alert raised when too many different messages were registered. */
  private static ConsoleAlert alertOverflow;

  /**
   * Raises or clears an alert straight from a {@code periodic()} method, without declaring a
   * field for it.
   * <p>
   * The alert is created on the first call as a {@link AlertType#kWarning} that stays on the
   * Elastic dashboard until it is dismissed manually, and is then reused on every following
   * call with the same message. Safe to call every loop: it only writes to the log and sends
   * an Elastic notification when {@code active} changes from false to true.
   * </p>
   *
   * <pre>{@code
   * public void periodic() {
   *   Log.alert("Arm motor is hot", motor.getTemperature() > 80);
   * }
   * }</pre>
   *
   * <p>
   * The message is the identity of the alert, so it must stay constant between calls. Building
   * it from a changing value ({@code "temp is " + temp}) creates a new alert every loop - after
   * {@link ConsoleConstants#CONDITION_ALERT_LIMIT} different messages this is detected and a
   * single error alert is raised instead. Use {@link ConsoleAlert} directly when the text has
   * to change.
   * </p>
   *
   * @param message the alert text, and the key the alert is remembered by
   * @param active  whether the condition of the alert currently holds
   * @return the alert backing this message, so it can be configured further if needed
   */
  public static ConsoleAlert alert(String message, boolean active) {
    ConsoleAlert alert = conditionAlerts.get(message);

    if (alert == null) {
      if (conditionAlerts.size() >= ConsoleConstants.CONDITION_ALERT_LIMIT) {
        return alertOverflow(message);
      }
      alert = ConsoleAlert.warning(message).withNoAutoDismiss();
      conditionAlerts.put(message, alert);
    }

    alert.set(active);
    return alert;
  }

  /**
   * Handles the case of too many different messages passed to {@link #alert(String, boolean)},
   * which almost always means the message is built from a changing value.
   *
   * @param message the message that could not be registered
   * @return the shared overflow alert
   */
  private static ConsoleAlert alertOverflow(String message) {
    if (alertOverflow == null) {
      alertOverflow = ConsoleAlert.error("Too many Log.alert messages")
          .withDescription("More than " + ConsoleConstants.CONDITION_ALERT_LIMIT
              + " different alert texts were registered. A message built from a changing value "
              + "creates a new alert every loop - keep the text constant, or use ConsoleAlert.")
          .withNoAutoDismiss();
      alertOverflow.set(true);
      DataLogManager.log("[Log.alert] message limit reached, ignoring: " + message);
    }
    return alertOverflow;
  }

  /**
   * Periodic method called by the scheduler.
   * Refreshes data, updates console alerts (handling expiration), and updates all
   * log entries.
   */
  @Override
  public void periodic() {
    Data.refreshAll();
    for (int i = activeConsole.size() - 1; i >= 0; i--) {
      ConsoleAlert alert = activeConsole.get(i);
      if (alert.isTimerOver()) {
        alert.set(false);
        activeConsole.remove(i);
      }
    }
    
    for (int i = 0; i < individualLogEntries.size(); i++) {
      individualLogEntries.get(i).log();
    }

    if (groupFloatEntry != null) {
      groupFloatEntry.log();
    }
    if (groupBooleanEntry != null) {
      groupBooleanEntry.log();
    }
    if (groupStringEntry != null) {
      groupStringEntry.log();
    }
    for (DashboardBuilder builder : builders.values()) {
            builder.pollInputs();
            builder.update();
        }
    
  }
    public static void putData(String key, Sendable sendable) {
        DashboardBuilder builder = builders.computeIfAbsent(key, DashboardBuilder::new);
        if (sendable instanceof NTSendable ntSendable) {
            ntSendable.initSendable(builder);
        } else {
            sendable.initSendable(builder);
        }
    }

  /**
   * Starts building a new log entry from Phoenix6 StatusSignals.
   * 
   * @param <T>           The type of data
   * @param name          The name of the log entry
   * @param statusSignals The signals to log
   * @return A new LogEntryBuilder
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, StatusSignal<T> statusSignal, boolean isRio) {
    return putData(name, new StatusSignal[] {statusSignal}, LogLevel.LOG_AND_NT, "", true, isRio);
  }

  /**
   * Starts building a new log entry from Suppliers.
   * 
   * @param <T>       The type of data
   * @param name      The name of the log entry
   * @param suppliers The suppliers to log
   * @return A new LogEntryBuilder
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, Supplier<T> supplier) {
    return putData(name, new Supplier[] {supplier}, LogLevel.LOG_AND_NT, "", true);
  }

  /**
   * Starts building a new log entry from data.
   * 
   * @param <T>       The type of data
   * @param name      The name of the log entry
   * @param data The data to log
   * @return A new LogEntryBuilder
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, Data<T> data) {
    return putData(name, new Data[] {data}, LogLevel.LOG_AND_NT, "", true);
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, StatusSignal<T>[] statusSignals, LogLevel logLevel, String metaData, boolean isSeparated, boolean isRio) {
    Data<T>[] data;
    data = (Data<T>[]) new Data[statusSignals.length];
    for (int i = 0; i < statusSignals.length; i++) {
        data[i] = new Data<>(statusSignals[i], isRio);
    }
    return putData(name, data, logLevel, metaData, isSeparated);
  }

  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, Supplier<T>[] suppliers, LogLevel logLevel, String metaData, boolean isSeparated) {
    Data<T>[] data;
    data = (Data<T>[]) new Data[suppliers.length];
    for (int i = 0; i < suppliers.length; i++) {
        data[i] = new Data<>(suppliers[i]);
    }
    return putData(name, data, logLevel, metaData, isSeparated);
  }

  /**
   * Internal method to add a log entry to the manager.
   * 
   * @param <T>         The data type
   * @param name        Name of the entry
   * @param data        Data wrapper
   * @param logLevel    Logging level
   * @param metaData    Metadata
   * @param isSeparated Whether to force a separate entry
   * @return The created or updated LogEntry
   */
  @SuppressWarnings("unchecked")
  public static <T> LogEntry<T> putData(String name, Data<T>[] data, LogLevel logLevel, String metaData, boolean isSeparated) {
    LogEntry<T> entry = null;

    if (isSeparated && data.length == 1) {
      entry = new LogEntry<T>(name, data[0], logLevel, metaData);
      logManager.individualLogEntries.add(entry);
    } else {
      if (data[0].isDouble()) {
        Data.addToGroupFloat(name, metaData, data);
        if (logManager.groupFloatEntry == null){
          logManager.groupFloatEntry = new LogEntry<float[]>(Data.getGroupFloatName(), () -> Data.getGroupFloat(), LogLevel.LOG_ONLY, Data.getGroupDoubleMetaData(), true, false);
        } else {
          logManager.groupFloatEntry.reInitialize(Data.getGroupFloatName(), () -> Data.getGroupFloat(), LogLevel.LOG_ONLY, Data.getGroupDoubleMetaData(), true, false);
        }
        entry = (LogEntry<T>) logManager.groupFloatEntry;
        
      } else if (data[0].isBoolean()) {
        Data.addToGroupBoolean(name, metaData, data);
        if (logManager.groupBooleanEntry == null){
          logManager.groupBooleanEntry = new LogEntry<boolean[]>(Data.getGroupBooleanName(), () -> Data.getGroupBoolean(), LogLevel.LOG_ONLY, Data.getGroupBooleanMetaData(), false, true);
        } else {
          logManager.groupBooleanEntry.reInitialize(Data.getGroupBooleanName(), () -> Data.getGroupBoolean(), LogLevel.LOG_ONLY, Data.getGroupBooleanMetaData(), false, true);
        }
        entry = (LogEntry<T>) logManager.groupBooleanEntry;
        
      } else {
        Data.addToGroupString(name, metaData, data);
        if (logManager.groupStringEntry == null){
          logManager.groupStringEntry = new LogEntry<String[]>(Data.getGroupStringName(), () -> Data.getGroupString(), LogLevel.LOG_ONLY, Data.getGroupStringMetaData(), false, false);
        } else {
          logManager.groupStringEntry.reInitialize(Data.getGroupStringName(), () -> Data.getGroupString(), LogLevel.LOG_ONLY, Data.getGroupStringMetaData(), false, false);
        }
        entry = (LogEntry<T>) logManager.groupStringEntry;
      }
    }

    return entry;
  }
}