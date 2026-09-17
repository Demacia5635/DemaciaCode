package frc.demacia.utils.log;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.demacia.utils.elastic.ElasticNotification;
import frc.demacia.utils.elastic.ElasticNotification.NotificationLevel;

/**
 * A wrapper for the WPILib {@link Alert} class that adds edge detection, a timer and an
 * optional Elastic pop-up notification.
 *
 * <p>The whole point of this class is that {@link #set(boolean)} is safe to call from a
 * {@code periodic()} method (50 times per second). The heavy work - writing to the log file and
 * publishing a notification to the Elastic dashboard - happens only on the rising edge of the
 * condition, meaning only on the loop where it changed from {@code false} to {@code true}.
 * As long as the condition stays {@code true} nothing else is sent, so NetworkTables, the
 * console and the Elastic connection are never flooded.
 *
 * <h2>Usage inside a mechanism / motor</h2>
 *
 * <pre>{@code
 * public class Arm extends SubsystemBase {
 *
 *   // Create the alert ONCE as a field - never inside periodic().
 *   // No need to set a description or a display time - they default to "" and 3 seconds.
 *   private final ConsoleAlert overheatAlert = ConsoleAlert.error("Arm motor overheating");
 *
 *   @Override
 *   public void periodic() {
 *     // Just hand it the condition every loop. The class handles the rest.
 *     overheatAlert.set(motor.getTemperature() > 80);
 *   }
 * }
 * }</pre>
 */
public class ConsoleAlert extends Alert {

  /** The Elastic alerts group this alert is published under. */
  public static final String DEFAULT_GROUP = "console";

  /** Default time the Elastic pop-up is shown, in seconds, unless overridden with {@link #withDisplaySeconds}. */
  public static final double DEFAULT_NOTIFICATION_SECONDS = 3;

  /**
   * Minimum time between two Elastic notifications of the same alert, in seconds.
   * Protects against a condition that rapidly toggles around its threshold.
   */
  public static final double DEFAULT_MIN_NOTIFICATION_INTERVAL = 1;

  /** The timer that tracks the duration of the alert */
  private final Timer timer = new Timer();

  private final AlertType type;
  private String text;

  /** Remembers the previous state, so the log and Elastic are only hit on the rising edge. */
  private boolean lastActive = false;

  private boolean notifyElastic;
  private String notificationTitle = null;
  private String notificationDescription = "";
  private double notificationSeconds = DEFAULT_NOTIFICATION_SECONDS;
  private double minNotificationInterval = DEFAULT_MIN_NOTIFICATION_INTERVAL;
  private double lastNotificationTime = Double.NEGATIVE_INFINITY;

  private boolean logOnRelease = false;

  /**
   * Creates an alert that shows up in the WPILib alerts widget, without an Elastic pop-up.
   *
   * @param text the alert text
   * @param type the severity of the alert
   */
  public ConsoleAlert(String text, AlertType type) {
    this(text, type, false);
  }

  /**
   * Creates an alert, optionally sending an Elastic pop-up on every activation.
   *
   * @param text the alert text
   * @param type the severity of the alert
   * @param notifyElastic whether to send an Elastic notification on the rising edge
   */
  public ConsoleAlert(String text, AlertType type, boolean notifyElastic) {
    super(DEFAULT_GROUP, text, type);
    this.text = text;
    this.type = type;
    this.notifyElastic = notifyElastic;
  }

  /**
   * Creates an informational alert with an Elastic pop-up enabled. The pop-up description
   * defaults to empty and its display time to {@value #DEFAULT_NOTIFICATION_SECONDS} seconds -
   * override them with {@link #withDescription} / {@link #withDisplaySeconds} only if needed.
   *
   * @param text the alert text, also used as the pop-up title
   * @return the new alert, for chaining
   */
  public static ConsoleAlert info(String text) {
    return new ConsoleAlert(text, AlertType.kInfo, true);
  }

  /**
   * Creates a warning alert with an Elastic pop-up enabled. The pop-up description defaults to
   * empty and its display time to {@value #DEFAULT_NOTIFICATION_SECONDS} seconds - override them
   * with {@link #withDescription} / {@link #withDisplaySeconds} only if needed.
   *
   * @param text the alert text, also used as the pop-up title
   * @return the new alert, for chaining
   */
  public static ConsoleAlert warning(String text) {
    return new ConsoleAlert(text, AlertType.kWarning, true);
  }

  /**
   * Creates an error alert with an Elastic pop-up enabled. The pop-up description defaults to
   * empty and its display time to {@value #DEFAULT_NOTIFICATION_SECONDS} seconds - override them
   * with {@link #withDescription} / {@link #withDisplaySeconds} only if needed.
   *
   * @param text the alert text, also used as the pop-up title
   * @return the new alert, for chaining
   */
  public static ConsoleAlert error(String text) {
    return new ConsoleAlert(text, AlertType.kError, true);
  }

  /**
   * Sets the title of the Elastic pop-up. Defaults to the alert text - only call this to show a
   * different title on the pop-up than the alert text.
   *
   * @param title the pop-up title
   * @return this alert, for chaining
   */
  public ConsoleAlert withTitle(String title) {
    this.notificationTitle = title;
    return this;
  }

  /**
   * Sets the description shown inside the Elastic pop-up - this is where the explanation of
   * what happened, and what the driver should do about it, belongs. Optional - defaults to
   * empty.
   *
   * @param description the pop-up description
   * @return this alert, for chaining
   */
  public ConsoleAlert withDescription(String description) {
    this.notificationDescription = description == null ? "" : description;
    return this;
  }

  /**
   * Sets how long the Elastic pop-up stays on screen. Optional - defaults to
   * {@value #DEFAULT_NOTIFICATION_SECONDS} seconds.
   *
   * @param seconds the number of seconds to display the pop-up
   * @return this alert, for chaining
   */
  public ConsoleAlert withDisplaySeconds(double seconds) {
    this.notificationSeconds = seconds;
    return this;
  }

  /**
   * Makes the Elastic pop-up stay on screen until the driver dismisses it manually.
   *
   * @return this alert, for chaining
   */
  public ConsoleAlert withNoAutoDismiss() {
    this.notificationSeconds = 0;
    return this;
  }

  /**
   * Enables or disables the Elastic pop-up for this alert.
   *
   * @param notifyElastic true to send a pop-up on every activation
   * @return this alert, for chaining
   */
  public ConsoleAlert withNotification(boolean notifyElastic) {
    this.notifyElastic = notifyElastic;
    return this;
  }

  /**
   * Sets the minimum time between two Elastic pop-ups of this alert. A condition that flickers
   * around its threshold can produce a rising edge every few loops, and this keeps that from
   * turning into a stream of pop-ups. Optional - defaults to
   * {@value #DEFAULT_MIN_NOTIFICATION_INTERVAL} second.
   *
   * @param seconds the minimum number of seconds between notifications
   * @return this alert, for chaining
   */
  public ConsoleAlert withMinNotificationInterval(double seconds) {
    this.minNotificationInterval = seconds;
    return this;
  }

  /**
   * Controls whether the alert writes a line to the log when the condition clears. Off by
   * default - turn it on for faults where knowing when they went away matters.
   *
   * @param logOnRelease true to write to the log on the falling edge
   * @return this alert, for chaining
   */
  public ConsoleAlert withLogOnRelease(boolean logOnRelease) {
    this.logOnRelease = logOnRelease;
    return this;
  }

  /**
   * Activates or deactivates the alert. Safe to call every loop from {@code periodic()}: the
   * log line and the Elastic pop-up are only produced when the state changes from inactive to
   * active.
   *
   * @param active whether the condition of the alert currently holds
   */
  @Override
  public void set(boolean active) {
    // The WPILib alerts widget (which Elastic displays) is updated every call - it is just a
    // boolean in NetworkTables, and WPILib itself ignores a repeated identical value.
    super.set(active);

    if (active) {
      if (!lastActive) {
        onActivate();
      }
      timer.start();
    } else {
      if (lastActive) {
        onRelease();
      }
      timer.stop();
      timer.reset();
    }

    lastActive = active;
  }

  /** Handles the rising edge - this runs exactly once per activation. */
  private void onActivate() {
    DataLogManager.log("[ALERT - " + type.name() + "] " + fullMessage());
    sendNotification();
  }

  /** Handles the falling edge - this runs exactly once per activation. */
  private void onRelease() {
    if (logOnRelease) {
      DataLogManager.log("[ALERT CLEARED - " + type.name() + "] " + fullMessage());
    }
  }

  /**
   * Sends the Elastic pop-up, unless notifications are disabled for this alert or the last one
   * was sent too recently.
   */
  private void sendNotification() {
    if (!notifyElastic) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastNotificationTime < minNotificationInterval) {
      return;
    }
    lastNotificationTime = now;

    ElasticNotification.sendNotification(
        new ElasticNotification.Notification()
            .withLevel(toNotificationLevel(type))
            .withTitle(getNotificationTitle())
            .withDescription(notificationDescription)
            .withDisplaySeconds(notificationSeconds));
  }

  /**
   * Translates a WPILib alert severity into the matching Elastic notification level.
   *
   * @param type the WPILib alert severity
   * @return the matching Elastic notification level
   */
  public static NotificationLevel toNotificationLevel(AlertType type) {
    switch (type) {
      case kError:
        return NotificationLevel.ERROR;
      case kWarning:
        return NotificationLevel.WARNING;
      default:
        return NotificationLevel.INFO;
    }
  }

  @Override
  public void setText(String text) {
    super.setText(text);
    this.text = text;
    timer.reset();
  }

  /**
   * @return the title used for the Elastic pop-up
   */
  public String getNotificationTitle() {
    return notificationTitle == null ? text : notificationTitle;
  }

  /** The text written to the log, combining the alert text and its description. */
  private String fullMessage() {
    return notificationDescription.isEmpty() ? text : text + " - " + notificationDescription;
  }

  /**
   * @return true once the alert has been shown for longer than
   *     {@link ConsoleConstants#CONSOLE_MESSEGE_TIME}, or false if that time is 0
   */
  public boolean isTimerOver() {
    if (ConsoleConstants.CONSOLE_MESSEGE_TIME == 0) {
      return false;
    }
    return timer.hasElapsed(ConsoleConstants.CONSOLE_MESSEGE_TIME);
  }
}