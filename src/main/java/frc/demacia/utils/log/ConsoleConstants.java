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
}