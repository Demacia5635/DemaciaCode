package frc.demacia.path;

public class demaciaTrajectory {
    private Leg[] legs;
    private double totalLength;

    public demaciaTrajectory(Leg[] legs) {
        this.legs = legs;
        this.totalLength = 0;
        for (Leg leg : legs) {
            this.totalLength += leg.getStart().getDistance(leg.getEnd());
        }
    }

    public Leg[] getLegs() {
        return legs;
    }

    public double getTotalLength() {
        return totalLength;
    }
}