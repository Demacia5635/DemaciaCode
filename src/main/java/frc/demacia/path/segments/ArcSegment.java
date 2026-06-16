package frc.demacia.path.segments;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ArcSegment extends SegmantBase {

    private Translation2d center;
    private Translation2d centerToStart;
    private Translation2d centerToFinish;
    private boolean isLeft;


    public ArcSegment(Translation2d startingPoint, Translation2d finishPoint, Translation2d centerCircle, boolean isLeft, double radius){
        super(startingPoint, finishPoint);

        this.center = centerCircle;
        this.isLeft = isLeft;

        this.centerToStart = startingPoint.minus(centerCircle);
        this.centerToFinish = finishPoint.minus(centerCircle);
    }

    public Translation2d getCenter(){return this.center;}
    
    public boolean getIsLeft() {return this.isLeft;}

    public Rotation2d getAngleBetweenRadius(){
        return centerToStart.getAngle().minus(centerToFinish.getAngle());
    }

    @Override
    public String toString(){
        return super.toString() + " Center Circle: " + this.center;
    }
}