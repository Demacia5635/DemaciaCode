package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.demacia.path.segmants.Segmant;

public class test {
    public static void main(String[] args) {
        Path path = new Path(new Pose2d[] {
            new Pose2d(0,0, new Rotation2d()), 
            new Pose2d(2.5,10, new Rotation2d()), 
            new Pose2d(5,1, new Rotation2d()), 
            new Pose2d(11.25,0.5, new Rotation2d()), 
            new Pose2d(15,5, new Rotation2d())
        });

        for (Segmant segmant : path.getSegmants()) {
            System.out.println(segmant.getStartPoint().getPointX() + " SSSSSStartPointXXXXX");
            System.out.println(segmant.getStartPoint().getPointY() + " SSSSStartPointYYYYY");
            System.out.println(segmant.getEndPoint().getPointX() + " EEEEEndPointXXXXX");
            System.out.println(segmant.getEndPoint().getPointY() + " EEEEEndPointYYYYY");
        }
    }
}
