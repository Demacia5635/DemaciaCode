package frc.demacia.utils.chassis;

public enum Mk5nConstants {
    R1(7.03, 
        0.16),
    R2(6.03, 
        0.19),
    R3(5.72, 
        0.2);

    public static final double WHEEL_DIAMETER = 4 * 0.0254;
    public static final double STEER_GEAR_RATIO = 287d / 11d;
    public final double metersFrom360Degs;
    public final double driveGearRatio;

    private Mk5nConstants(double driveGearRatio, double metersFrom360Degs) {
        this.driveGearRatio = driveGearRatio;
        this.metersFrom360Degs = metersFrom360Degs;
    }
}
