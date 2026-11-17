package frc.demacia.utils.log;

public class ConsoleConstants {
  public static double  CONSOLE_MESSEGE_TIME = 5;
  public static final int CONSOLE_LIMIT = 10;

  /**
   * Maximum number of different messages {@link Log#alert(String, boolean)} will register.
   * Every distinct message creates one ConsoleAlert that lives for the whole match, so a
   * message built from a changing value would otherwise leak a new alert every loop.
   */
  public static final int CONDITION_ALERT_LIMIT = 50;

  /**
   * Seconds Elastic must be connected before the pop-ups held by {@link Log#alert} are sent. Elastic subscribes to the notification topic a moment after it connects, so a pop-up
   * sent at the instant of connection would be missed.
   */
  public static final double ALERT_POPUP_CONNECT_DELAY = 2;

  /**
   * Start of the NetworkTables client name Elastic connects with (it shows up as "Elastic@1").
   * Other clients such as a Limelight or AdvantageScope do not show pop-ups, so only a client
   * with this name counts as a reason to send them.
   */
  public static final String ELASTIC_CLIENT_NAME_PREFIX = "Elastic";
}