package frc.robot.logTesting;

import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.log.LogEntryBuilder.LogLevel;

public class LogFunctions {
public LogFunctions() {
    logTest();
    
}
double[] testArray1 = {1.0, 2.0, 3.0};

    public void logTest() {
        java.util.function.Supplier<Double> doubleSupplier = () -> Double.valueOf(1.0);
        java.util.function.Supplier<String> stringSupplier = () -> "something";
        java.util.function.Supplier<Boolean> booleanSupplier = () -> Boolean.valueOf(true);

        LogManager.addEntry("double", doubleSupplier).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
        LogManager.addEntry("string", stringSupplier).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
        LogManager.addEntry("boolean", booleanSupplier).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).build();
        LogManager.addEntry("testArray1", () -> testArray1).withLogLevel(LogLevel.LOG_AND_NT_NOT_IN_COMP).withIsSeparated(true).build();
    }

}