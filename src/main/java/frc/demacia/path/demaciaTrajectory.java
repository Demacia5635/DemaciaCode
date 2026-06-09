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
    
    private List<Circle> allCircles;
    private List<Leg> allLegs;
    
    private double radius = 0.5;

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

        allCircles.add(new Circle(demaciaPathPoint.get(0).getTranslation(), 0, false, 0));

        for (int i = 1; i < demaciaPathPoint.size() - 1; i++) {
            Translation2d from = demaciaPathPoint.get(i - 1).getTranslation();
            Translation2d mid = demaciaPathPoint.get(i).getTranslation();
            Translation2d to = demaciaPathPoint.get(i + 1).getTranslation();
            
            Circle circle = CircleCalculator.calculateCircleCenter(from, to, mid, this.radius);

            LogManager.log("form: " + from + " mid: " + mid + " to: " + to + " circle raduis: " + circle.radius + " circle center: " + circle.center);

            allCircles.add(circle);
        }

        allCircles.add(new Circle(demaciaPathPoint.get(demaciaPathPoint.size() - 1).getTranslation(), 0, false, 0));

        // 3. Build straight legs between circles
        for (int i = 1; i < allCircles.size(); i++) {            
            Circle startCircle = allCircles.get(i - 1);
            Circle endCircle = allCircles.get(i);
            
            Leg leg = new Leg(startCircle, endCircle);
            allLegs.add(leg);
            
            this.pathPoint.add(new Pose2d(leg.getStart(), demaciaPathPoint.get(i - 1).getRotation()));
            this.pathPoint.add(new Pose2d(leg.getEnd(), demaciaPathPoint.get(i).getRotation()));
        }

        for (int i = 0; i < this.pathPoint.size(); i++){
            LogManager.log("Path Point " + i + ": " + this.pathPoint.get(i));
        }

        LogManager.log("Path Point size: " + this.pathPoint.size());
        LogManager.log("All Circles size: " + this.allCircles.size());
        LogManager.log("All Legs size: " + this.allLegs.size());
        LogManager.log("Demacia Path Point size: " + this.demaciaPathPoint.size());
        LogManager.log("lest path point" + getPointLestPose());
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
            // Off-track or uninitialized: safe stop
            // LogManager.log("fauck");
            LogManager.log("location: " + location);
            return new ChassisSpeeds(0, 0, 0);
        }

        // Convert scalar speed and heading into field-relative X and Y velocities
        LogManager.log("target vel: " + targetVelocity + " target heading: " + targetHeading);
        double vx = targetVelocity * targetHeading.getCos();
        double vy = targetVelocity * targetHeading.getSin();
        
        // P-control loop for angular velocity targeting the desired heading
        double headingError = targetHeading.minus(currentRobotPose.getRotation()).getRadians();
        double omega = headingError * 4.0; 

        return new ChassisSpeeds(vx, vy, omega);
    }

    
    public Object checkRobotLocation(Pose2d currentRobotPose) {
        // Safety margin in meters
        double tolerance = 0.7; 

        // First priority: Check if the robot is inside any of the defined turning circles (with 0.5m tolerance)
        for (Circle circle : allCircles){
            if (isInsideCircleWithTolerance(currentRobotPose, circle, tolerance)){
                return circle;
            }
        }

        // Second priority: Check if the robot is tracking along a straight leg segment
        for (Leg leg : allLegs) {
            if (isRobotNearLine(currentRobotPose, leg)) {
                return leg;
            }
        }

        // If the robot is neither in the extended circle nor on the line, it's off-track
        return "ROBOT_LOST_BETWEEN_POINTS"; 
    }

    /**
     * Helper method to check if the robot is within the circle's radius plus a tolerance margin.
     */
    private boolean isInsideCircleWithTolerance(Pose2d robotPose, Circle circle, double tolerance) {
        // Calculate physical distance from the robot to the center of the circle
        double distanceToCenter = robotPose.getTranslation().minus(circle.center).getNorm();
        
        // Return true if the robot is within the radius + tolerance threshold
        return distanceToCenter <= (circle.radius + tolerance);
    }

    /**
     * Calculates cross-track error from the robot to the finite line segment of the leg.
     */
    private boolean isRobotNearLine(Pose2d robotPose, Leg leg) {
        Translation2d start = leg.getStart();
        Translation2d end = leg.getEnd();
        Translation2d robot = robotPose.getTranslation();

        Translation2d segment = end.minus(start);
        Translation2d robotToStart = robot.minus(start);

        double segmentLengthSq = segment.getX() * segment.getX() + segment.getY() * segment.getY();
        if (segmentLengthSq == 0) return false;

        // Calculate projection factor 't' and clamp it to keep it within the segment boundaries
        double t = (robotToStart.getX() * segment.getX() + robotToStart.getY() * segment.getY()) / segmentLengthSq;
        t = Math.max(0.0, Math.min(1.0, t));

        // Get the closest point on the segment
        Translation2d closestPoint = start.plus(new Translation2d(t * segment.getX(), t * segment.getY()));

        // Verify if the distance is within the 0.2 meters tolerance threshold
        double distance = robot.getDistance(closestPoint);
        return distance <= 0.2; 
    }

    public Pose2d getPointLestPose(){
        return demaciaPathPoint.get(demaciaPathPoint.size() - 1);
    }

    public List<Pose2d> getDemaciaPathPoint() {
        return demaciaPathPoint;
    }
}