package frc.demacia.path;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.demacia.utils.log.LogManager;

public class demaciaTrajectory {
    private List<Pose2d> demaciaPathPoint;
    private List<PointPair<Pose2d>> pointPairs;
    private List<Pose2d> pathPoint;
    
    private List<CircleCalculator> allCircles;
    private List<Leg> allLegs;
    
    private double radius = 3.0;

    public demaciaTrajectory(List<Pose2d> demaciaPathPoint) {
        this.demaciaPathPoint = demaciaPathPoint;
        this.pointPairs = new ArrayList<>();
        this.pathPoint = new ArrayList<>();
        this.allCircles = new ArrayList<>();
        this.allLegs = new ArrayList<>();
        
        // 1. Generate unique combinations of pairs
        for (int i = 0; i < demaciaPathPoint.size() - 1; i++) {
            for (int j = i + 1; j < demaciaPathPoint.size(); j++) {
                PointPair<Pose2d> pointPair = new PointPair<>(demaciaPathPoint.get(i), demaciaPathPoint.get(j));
                this.pointPairs.add(pointPair);
            }
        }

        // 2. Compute turn circles for middle waypoints
        for (int i = 1; i < demaciaPathPoint.size() - 1; i++) {
            Translation2d from = demaciaPathPoint.get(i - 1).getTranslation();
            Translation2d mid = demaciaPathPoint.get(i).getTranslation();
            Translation2d to = demaciaPathPoint.get(i + 1).getTranslation();
            
            CircleCalculator circle = CircleCalculator.calculateCircleCenter(from, to, mid, this.radius);
            allCircles.add(circle);
        }

        // 3. Build straight legs between circles
        for (int i = 1; i < allCircles.size(); i++) {
            CircleCalculator c1 = allCircles.get(i - 1);
            CircleCalculator c2 = allCircles.get(i);
            
            Circle startCircle = new Circle(c1.center, c1.radius, c1.isLeftTurn);
            Circle endCircle = new Circle(c2.center, c2.radius, c2.isLeftTurn);
            
            Leg leg = new Leg(startCircle, endCircle);
            allLegs.add(leg);
            
            this.pathPoint.add(new Pose2d(leg.getStart(), demaciaPathPoint.get(i - 1).getRotation()));
            this.pathPoint.add(new Pose2d(leg.getEnd(), demaciaPathPoint.get(i).getRotation()));
        }

    }

    /**
     * Evaluates where the robot is and computes linear/rotational target ChassisSpeeds.
     */
    public ChassisSpeeds getChassisSpeeds(Pose2d currentRobotPose) {
        Object location = checkRobotLocation(currentRobotPose);
        
        double targetVelocity = 0;
        Rotation2d targetHeading = currentRobotPose.getRotation(); // Fallback to current heading

        if (location instanceof Circle) {
            Circle currentCircle = (Circle) location;
            
            currentCircle.updateDistance(currentRobotPose);
            targetVelocity = currentCircle.getVel();
            
            Translation2d baseVector = currentCircle.center.minus(currentRobotPose.getTranslation());
            targetHeading = currentCircle.getTargetHeading(currentRobotPose.getRotation(), baseVector);
            
        } else if (location instanceof Leg) {
            Leg currentLeg = (Leg) location;
            
            currentLeg.updateDistance(currentRobotPose);
            targetVelocity = currentLeg.getVel();
            
            Translation2d baseVector = currentLeg.getEnd().minus(currentRobotPose.getTranslation());
            targetHeading = currentLeg.getTargetHeading(currentRobotPose.getRotation(), baseVector);
        } else {
            // Off-track: safe stop
            return new ChassisSpeeds(0, 0, 0);
        }

        // Turn scalar speed and rotation into linear components (X and Y velocities)
        double vx = targetVelocity * targetHeading.getCos();
        double vy = targetVelocity * targetHeading.getSin();
        
        // P-control for angular velocity targeting the desired tracking heading
        double headingError = targetHeading.minus(currentRobotPose.getRotation()).getRadians();
        double omega = headingError * 4.0; 

        return new ChassisSpeeds(vx, vy, omega);
    }

    public Object checkRobotLocation(Pose2d currentRobotPose) {
        for (Leg leg : allLegs) {
            if (leg.getStartCircle().isPointInside(currentRobotPose)) {
                return leg.getStartCircle(); 
            }
            if (leg.getEndCircle().isPointInside(currentRobotPose)) {
                return leg.getEndCircle();   
            }
        }

        for (Leg leg : allLegs) {
            if (isRobotNearLine(currentRobotPose, leg)) {
                return leg;
            }
        }
        return null;
    }

    private boolean isRobotNearLine(Pose2d robotPose, Leg leg) {
        double x = robotPose.getX();
        double y = robotPose.getY();
        double x1 = leg.getStartCircle().center.getX();
        double y1 = leg.getStartCircle().center.getY();
        double x2 = leg.getEndCircle().center.getX();
        double y2 = leg.getEndCircle().center.getY();

        double num = Math.abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1);
        double den = Math.sqrt(Math.pow(y2 - y1, 2) + Math.pow(x2 - x1, 2));
        return (num / den) <= 0.2; 
    }

    public Pose2d getPointLestPose(){
        return this.demaciaPathPoint.get(demaciaPathPoint.size() - 1);
    }

    public List<Pose2d> getDemaciaPathPoint() {
        return demaciaPathPoint;
    }
}