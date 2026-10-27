package frc.demacia.path;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.demacia.path.segmants.Segmant;

public class test {
    public static void main(String[] args) {
        Path path = new Path(new Pose2d[] {
            new Pose2d(0,0, new Rotation2d()), 
            new Pose2d(10,40, new Rotation2d()), 
            new Pose2d(20,4, new Rotation2d()), 
            new Pose2d(45,2, new Rotation2d()), 
            new Pose2d(60,20, new Rotation2d())
        });

        for (Segmant segmant : path.getSegmants()) {
            System.out.println(segmant.getStartPoint().getPointX() + " SSSSSStartPointXXXXX");
            System.out.println(segmant.getStartPoint().getPointY() + " SSSSStartPointYYYYY");
            System.out.println(segmant.getEndPoint().getPointX() + " EEEEEndPointXXXXX");
            System.out.println(segmant.getEndPoint().getPointY() + " EEEEEndPointYYYYY");
        }
    }
}
