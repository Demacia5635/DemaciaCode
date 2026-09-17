package frc.demacia.utils.log;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;

/**
 * A wrapper for the WPILib Alert class that includes a timer and prevents log spamming.
 */
public class ConsoleAlert extends Alert {
  
  /** The timer that tracks the duration of the alert */
  private final Timer timer;
  
  private final String initialText;
  private final AlertType type;
  
  // הזיכרון שמונע הדפסת לוגים 50 פעם בשנייה
  private boolean lastActive = false; 

  public ConsoleAlert(String text, AlertType type) {
    super("console", text, type); // "console" זו הקבוצה שתראה באלסטיק
    this.initialText = text;
    this.type = type;
    this.timer = new Timer();
  }

  @Override
  public void set(boolean active) {
    // 1. קורא ל-WPILib הרגיל (ואלסטיק כבר יזהה את זה ויקפיץ פופ-אפ!)
    super.set(active);

    // 2. מנגנון למניעת הספמת הלוג
    if (active) {
        if (!lastActive) {
            // כאן מדפיסים לקונסול רק בפעם הראשונה שהתקלה קורית
            System.out.println("[ALERT - " + this.type.name() + "] " + this.initialText);
        }
        timer.start();
    } else {
        timer.stop();
        timer.reset();
    }
    
    // 3. עדכון הזיכרון
    lastActive = active;
  }

  @Override
  public void setText(String text) {
    super.setText(text);
    timer.reset();
  }

  public boolean isTimerOver() {
    if (ConsoleConstants.CONSOLE_MESSEGE_TIME == 0) {
      return false;
    }
    return timer.hasElapsed(ConsoleConstants.CONSOLE_MESSEGE_TIME);
  }
}