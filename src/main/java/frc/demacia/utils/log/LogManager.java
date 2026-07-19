// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.demacia.utils.log;

import java.util.ArrayList;
import java.util.function.Supplier;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.Data;
import frc.demacia.utils.RobotCommon;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;
import frc.demacia.utils.sysid.sysidCommand;

/**
 * Centralized logging system for robot telemetry and diagnostics.
 * <p>
 * Manages the creation, updating, and optimization of log entries.
 * Handles both file logging (DataLog) and live dashboard updates
 * (NetworkTables).
 * </p>
 */
public class LogManager extends SubsystemBase {

  /** Singleton instance of the LogManager */
  private static LogManager logManager;

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
  
  double millis;
  long count;
  int warmupCount;

  /**
   * Private constructor to enforce Singleton pattern.
   * Initializes DataLogManager and starts logging.
   */
  private LogManager() {
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

    SmartDashboard.putData("sysID/sysidCommand", new sysidCommand());
    RobotCommon.init();
  }

  /**
   * Static initializer to ensure the LogManager is created.
   */
  static {
    if (logManager == null) {
      new LogManager();
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
  public static <T> LogEntryBuilder<T> addEntry(String name, StatusSignal<T>... statusSignals) {
    return new LogEntryBuilder<T>(name, statusSignals);
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
  public static <T> LogEntryBuilder<T> addEntry(String name, Supplier<T>... suppliers) {
    return new LogEntryBuilder<T>(name, suppliers);
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
  public static <T> LogEntryBuilder<T> addEntry(String name, Data<T>... data) {
    return new LogEntryBuilder<T>(name, data);
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
   * Periodic method called by the scheduler.
   * Refreshes data, updates console alerts (handling expiration), and updates all
   * log entries.
   */
  @Override
  public void periodic() {
    long start = System.nanoTime();
    Data.refreshAll();
    long end = System.nanoTime();
    millis += (end - start) / 1e6;
    count++;
    SmartDashboard.putNumber("refreshAll Periodic Time ms", millis / count);

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
  public static <T> LogEntry<T> add(String name, Data<T>[] data, LogLevel logLevel, String metaData, boolean isSeparated,
      boolean isRio) {
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