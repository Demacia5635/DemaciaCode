package frc.demacia.utils.mechanisms;

import java.util.HashMap;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.demacia.utils.LookUpTable;
import frc.demacia.utils.log.LogManager;
import frc.demacia.utils.motors.MotorInterface;
import frc.demacia.utils.sensors.SensorInterface;

/**
 * A base class for robot mechanisms (subsystems) that manage a collection of motors and sensors.
 * <p>
 * This class provides common functionality for:
 * <ul>
 * <li>Storing motors and sensors by name for easy retrieval.</li>
 * <li>Controlling all motors at once (stop, set power, set neutral mode).</li>
 * <li>Automatically creating SmartDashboard buttons for switching Neutral Modes (Brake/Coast).</li>
 * <li>Performing electronics checks on hardware.</li>
 * </ul>
 * </p>
 */
public class BaseMechanism extends SubsystemBase{
    /** The name of the mechanism (used for logging and dashboard) */
    protected String name;
    /** Map of motors belonging to this mechanism, keyed by their name */
    protected HashMap<String, MotorInterface> motors;
    /** Map of sensors belonging to this mechanism, keyed by their name */
    protected HashMap<String, SensorInterface> sensors;
    /** Map of motors limits belonging to this mechanism, keyed by their name */
    protected HashMap<String, Pair<Double, Double>> motorLimits;

    protected MotorInterface[] motorArray;
    protected SensorInterface[] sensorArray;
    protected Pair<Double,Double>[] motorLimitsArray;

    protected boolean hasCalibrated = true;

    LookUpTable lookUpTable;
    DoubleSupplier distance;

    /**
     * Constructs a new BaseMechanism.
     * Initializes the motor and sensor maps and creates debug buttons on the Dashboard.
     * @param name The name of the subsystem
     * @param motors Array of motors to register
     * @param sensors Array of sensors to register
     */
    @SuppressWarnings("unchecked")
    public BaseMechanism(String name, MotorInterface[] motors, SensorInterface[] sensors) {
        this.name = name;
        setName(name);
        motorArray = motors;
        sensorArray = sensors;
        // Initialize motors map
        this.motors = new HashMap<>();
        for (MotorInterface motor : motors) {
            this.motors.put(motor.getName(), motor);
        }
        
        // Initialize sensors map
        this.sensors = new HashMap<>();
        for (SensorInterface sensor : sensors) {
            this.sensors.put(sensor.getName(), sensor);
        }
        
        // Initialize motors limits
        motorLimits = new HashMap<>();
        motorLimitsArray = new Pair[motors.length];
        for (int i = 0; i < motors.length; i++){
            motorLimitsArray[i] = new Pair<Double,Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            motorLimits.put(motors[i].getName(), new Pair<Double,Double>(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY));
        }

        // Create individual Brake/Coast buttons for each motor
        for (String motorName : this.motors.keySet()) {
            SmartDashboard.putData(getName() + "/" + motorName + "/set brake", 
                new InstantCommand(() -> setNeutralMode(motorName, true)).ignoringDisable(true));
            SmartDashboard.putData(getName() + "/" + motorName + "/set coast", 
                new InstantCommand(() -> setNeutralMode(motorName, false)).ignoringDisable(true));
        }

        // Create global Brake/Coast buttons for the whole mechanism
        SmartDashboard.putData(getName() + "/set coast all", 
                new InstantCommand(() -> setNeutralMode(false)).ignoringDisable(true));
        SmartDashboard.putData(getName() + "/set brake all", 
                new InstantCommand(() -> setNeutralMode(true)).ignoringDisable(true));
        
        SmartDashboard.putData(name, this);
    }

    /**
     * @return The name of the mechanism
     */
    public String getName(){
        return name;
    }
    
    /**
     * Marks that this mechanism requires calibration.
     * Sets the calibration status to false.
     */
    public void withCalibration(){
        hasCalibrated = false;
    }

    /**
     * @return true if the mechanism is calibrated and ready for control, false otherwise.
     */
    public boolean getCalibration(){
        return hasCalibrated;
    }

    /**
     * Sets the calibration status of the mechanism.
     * @param hasCalibrated true if calibrated, false otherwise.
     */
    public void setCalibration(boolean hasCalibrated){
        this.hasCalibrated = hasCalibrated;
    }

    /**
     * Attaches a lookup table and a distance source to the mechanism.
     * @param lookUpTable The table for interpolation.
     * @param distance A supplier for the input value (e.g., limelight distance).
     */
    public void withLookUpTable(LookUpTable lookUpTable, DoubleSupplier distance){
        this.lookUpTable = lookUpTable;
        this.distance = distance;
    }

    /**
     * Interpolates all values from the lookup table based on current distance.
     * @return Array of interpolated values, or empty array if table not set.
     */
    public double[] getLookUpTableValues(){
        if (lookUpTable == null){
            LogManager.log("you didn't set the lookUpTable");
            return new double[0];
        }
        return lookUpTable.get(distance.getAsDouble());
    }

    /**
     * Gets a specific interpolated value from the lookup table.
     * @param i The index of the output value.
     * @return The interpolated value at index i.
     */
    public double getLookUpTableValue(int i){
        if (lookUpTable == null){
            LogManager.log("you didn't set the lookUpTable");
            return 0;
        }
        return lookUpTable.get(distance.getAsDouble())[i];
    }

    public void addLimit(String motorName, double min,  double max) {
        motorLimits.put(motorName, new Pair<Double, Double>(min, max));
    }

    public void addLimit(int motorIndex, double min,  double max) {
        motorLimitsArray[motorIndex] = new Pair<Double, Double>(min, max);
    }

    public void addLimitMax(String motorName, double max) {
        motorLimits.put(motorName, new Pair<Double, Double>(
            motorLimits.get(motorName).getFirst(), 
            max));
    }

    public void addLimitMax(int motorIndex, double max) {
        motorLimitsArray[motorIndex] = new Pair<Double, Double>(
            motorLimitsArray[motorIndex].getFirst(), 
            max);
    }

    public void addLimitMin(String motorName, double min) {
        motorLimits.put(motorName, new Pair<Double, Double>(
            min,
            motorLimits.get(motorName).getSecond()));
    }

    public void addLimitMin(int motorIndex, double min) {
        motorLimitsArray[motorIndex] = new Pair<Double, Double>(
            min,
            motorLimitsArray[motorIndex].getSecond());
    }

    /**
     * Stops all motors in this mechanism.
     */
    public void stop(){
        if (motors == null) return;
        for (MotorInterface motor : motors.values()){
            motor.stop();
        }
    }

    /**
     * Stops a specific motor by name.
     * @param motorName The name of the motor to stop
     */
    public void stop(String motorName){
        if (isValidMotor(motorName)){
            motors.get(motorName).stop();
        }
    }

    /**
     * Stops a specific motor by index.
     * @param motorIndex The index of the motor to stop
     */
    public void stop(int motorIndex){
        if (isValidMotor(motorIndex)){
            motorArray[motorIndex].stop();
        }
    }

    /**
     * Sets the duty cycle (power) for all motors.
     * @param power The power to set [-1.0, 1.0]
     */
    public void setPowerAll(double power) {
        if (motors == null || !hasCalibrated) return;
        for (MotorInterface motor : motors.values()){
            motor.setDuty(power);
        }
    }

    /**
     * Sets the duty cycle (power) for a specific motor.
     * @param motorName The name of the motor
     * @param power The power to set [-1.0, 1.0]
     */
    public void setPower(String motorName, double power){
        if (canMove(motorName)){
            motors.get(motorName).setDuty(power);
        }
    }

    /**
     * Sets the duty cycle (power) for a specific motor.
     * @param motorIndex The index of the motor
     * @param power The power to set [-1.0, 1.0]
     */
    public void setPower(int motorIndex, double power){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setDuty(power);
        }
    }

    /**
     * Sets the Voltage for a specific motor.
     * @param motorName The name of the motor
     * @param voltage The Voltage to set
     */
    public void setVoltage(String motorName, double voltage){
        if (canMove(motorName)){
            motors.get(motorName).setVoltage(voltage);
        }
    }

    /**
     * Sets the Voltage for a specific motor.
     * @param motorIndex The index of the motor
     * @param voltage The Voltage to set
     */
    public void setVoltage(int motorIndex, double voltage){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setVoltage(voltage);
        }
    }

    /**
     * Sets the Velocity for a specific motor.
     * @param motorName The name of the motor
     * @param velocity The Velocity to set
     */
    public void setVelocity(String motorName, double velocity){
        if (canMove(motorName)){
            motors.get(motorName).setVelocity(velocity);
        }
    }

    /**
     * Sets the Velocity for a specific motor.
     * @param motorIndex The index of the motor
     * @param velocity The Velocity to set
     */
    public void setVelocity(int motorIndex, double velocity){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setVelocity(velocity);
        }
    }

    /**
     * Sets the position using PositionVoltage for a specific motor.
     * @param motorName The name of the motor
     * @param position The position to set
     */
    public void setPositionVoltage(String motorName, double position){
        if (canMove(motorName)){
            motors.get(motorName).setPositionVoltage(clampInLimits(motorName, position));
        }
    }

    /**
     * Sets the position using PositionVoltage for a specific motor.
     * @param motorIndex The index of the motor
     * @param position The position to set
     */
    public void setPositionVoltage(int motorIndex, double position){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setPositionVoltage(clampInLimits(motorIndex, position));
        }
    }

    /**
     * Sets the position for a specific motor.
     * @param motorName The name of the motor
     * @param position The position to set
     */
    public void setMotion(String motorName, double position){
        if (canMove(motorName)){
            motors.get(motorName).setMotion(clampInLimits(motorName, position));
        }
    }

    /**
     * Sets the position for a specific motor.
     * @param motorIndex The index of the motor
     * @param position The position to set
     */
    public void setMotion(int motorIndex, double position){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setMotion(clampInLimits(motorIndex, position));
        }
    }

    /**
     * Sets the Angle for a specific motor.
     * @param motorName The name of the motor
     * @param angle The Angle to set
     */
    public void setAngle(String motorName, double angle){
        if (canMove(motorName)){
            motors.get(motorName).setAngle(clampInLimits(motorName, angle));
        }
    }

    /**
     * Sets the Angle for a specific motor.
     * @param motorIndex The index of the motor
     * @param angle The Angle to set
     */
    public void setAngle(int motorIndex, double angle){
        if (canMove(motorIndex)){
            motorArray[motorIndex].setAngle(clampInLimits(motorIndex, angle));
        }
    }

    private boolean canMove(String motorName) {
        return isValidMotor(motorName) && hasCalibrated;
    }

    private boolean canMove(int motorIndex) {
        return isValidMotor(motorIndex) && hasCalibrated;
    }

    private double clampInLimits(String motorName,double position) {
        return MathUtil.clamp(position, motorLimits.get(motorName).getFirst(), motorLimits.get(motorName).getSecond());
    }

    private double clampInLimits(int motorIndex,double position) {
        return MathUtil.clamp(position, motorLimitsArray[motorIndex].getFirst(), motorLimitsArray[motorIndex].getSecond());
    }

    /**
     * Sets the neutral mode (Brake or Coast) for all motors.
     * @param isBrake true for Brake mode, false for Coast mode
     */
    public void setNeutralMode(boolean isBrake) {
        if (motors == null) return;
        for (MotorInterface motor : motors.values()) {
            if (motor != null) motor.setNeutralMode(isBrake);
        }
    }

    /**
     * Sets the neutral mode (Brake or Coast) for a specific motor.
     * @param motorName The name of the motor
     * @param isBrake true for Brake mode, false for Coast mode
     */
    public void setNeutralMode(String motorName, boolean isBrake){
        if (isValidMotor(motorName)){
            motors.get(motorName).setNeutralMode(isBrake);
        }
    }

    /**
     * Sets the neutral mode (Brake or Coast) for a specific motor.
     * @param motorIndex The index of the motor
     * @param isBrake true for Brake mode, false for Coast mode
     */
    public void setNeutralMode(int motorIndex, boolean isBrake){
        if (isValidMotor(motorIndex)){
            motorArray[motorIndex].setNeutralMode(isBrake);
        }
    }

    /**
     * Triggers the electronics check for all motors and sensors.
     */
    public void checkElectronics() {
        if (motors == null) return;
        for (MotorInterface motor : motors.values()) {
            if (motor != null) motor.checkElectronics();
        }
        if (sensors == null) return;
        for (SensorInterface sensor : sensors.values()) {
            if (sensor != null) sensor.checkElectronics();
        }
    }

    /**
     * Checks electronics for a specific motor.
     * @param motorName The name of the motor
     */
    public void checkElectronicsMotor(String motorName){
        if (isValidMotor(motorName)){
            motors.get(motorName).checkElectronics();
        }
    }

    /**
     * Checks electronics for a specific motor.
     * @param motorIndex The index of the motor
     */
    public void checkElectronicsMotor(int motorIndex){
        if (isValidMotor(motorIndex)){
            motorArray[motorIndex].checkElectronics();
        }
    }

    /**
     * Checks electronics for a specific sensor.
     * @param sensorName The name of the sensor
     */
    public void checkElectronicsSensor(String sensorName){
        if (isValidSensor(sensorName)){
            sensors.get(sensorName).checkElectronics();
        }
    }

    /**
     * Checks electronics for a specific sensor.
     * @param sensorName The index of the sensor
     */
    public void checkElectronicsSensor(int sensorIndex){
        if (isValidSensor(sensorIndex)){
            sensorArray[sensorIndex].checkElectronics();
        }
    }

    /**
     * Retrieves a motor object by its name.
     * Logs an error if the motor name is invalid.
     * @param motorName The name of the motor
     * @return The MotorInterface object, or null if not found
     */
    public MotorInterface getMotor(String motorName) {
        if (!isValidMotor(motorName)){
            LogManager.log("Invalid motor: " + motorName);
            return null;
        }
        return motors.get(motorName);
    }

    /**
     * Retrieves a motor object by its index.
     * Logs an error if the motor index is invalid.
     * @param motorIndex The index of the motor
     * @return The MotorInterface object, or null if not found
     */
    public MotorInterface getMotor(int motorIndex) {
        if (!isValidMotor(motorIndex)){
            LogManager.log("Invalid motor index: " + motorIndex);
            return null;
        }
        return motorArray[motorIndex];
    }

    public MotorInterface[] getMotors() {
        return motorArray;
    }

    /**
     * Retrieves a sensor object by its name.
     * Logs an error if the sensor name is invalid.
     * @param sensorName The name of the sensor
     * @return The SensorInterface object, or null if not found
     */
    public SensorInterface getSensor(String sensorName) {
        if (!isValidSensor(sensorName)){
            LogManager.log("Invalid sensor: " + sensorName);
            return null;
        }
        return sensors.get(sensorName);
    }

    /**
     * Retrieves a sensor object by its index.
     * Logs an error if the sensor index is invalid.
     * @param sensorIndex The index of the sensor
     * @return The SensorInterface object, or null if not found
     */
    public SensorInterface getSensor(int sensorIndex) {
        if (!isValidSensor(sensorIndex)){
            LogManager.log("Invalid sensor: " + sensorIndex);
            return null;
        }
        return sensorArray[sensorIndex];
    }

    /**
     * Checks if a motor name exists in the map.
     * @param motorName The name to check
     * @return true if valid, false otherwise
     */
    protected boolean isValidMotor(String motorName) {
        return motors.containsKey(motorName);
    }

    /**
     * Checks if a motor index exists.
     * @param motorIndex The index to check
     * @return true if valid, false otherwise
     */
    protected boolean isValidMotor(int motorIndex) {
        return motorIndex >= 0 && motorIndex < motorArray.length;
    }

    /**
     * Checks if a sensor name exists in the map.
     * @param sensorName The name to check
     * @return true if valid, false otherwise
     */
    protected boolean isValidSensor(String sensorName) {
        return sensors.containsKey(sensorName);
    }

    /**
     * Checks if a sensor index exists in the map.
     * @param sensorIndex The index to check
     * @return true if valid, false otherwise
     */
    protected boolean isValidSensor(int sensorIndex) {
        return sensorIndex >= 0 && sensorIndex < sensorArray.length;
    }
}