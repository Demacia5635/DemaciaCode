package frc.demacia.utils.sysid;

import java.util.ArrayList;
import java.util.List;
import org.ejml.simple.SimpleMatrix;
import frc.demacia.utils.log.Log;
import frc.demacia.utils.log.LogReader.Entry;
import frc.demacia.utils.log.LogReader.EntryPoint;
import frc.demacia.utils.motors.CloseLoopParam;
import frc.demacia.utils.motors.MotorInterface;

public class Sysid {
    private static final List<MotorInterface> motors = new ArrayList<>();

    private static final double[] VOLTAGE_THRESHOLDS = {0.1, 0.2, 0.3, 0.4};
    private static final int[] SMOOTH_WINDOWS = {1, 2, 3, 4, 5};
    private static final double[] Z_SCORE_THRESHOLDS = {1.5, 2.0, 2.5, 3.0};

    private KFlags kFlags = new KFlags(true, true, true, false, false, false);

    private String name;
    private List<Entry> motorEntries;

    private class SyncedDataPoint {
        double velocity, position, acceleration, rawAcceleration, voltage;
        long timestamp;
        double error;
        
        SyncedDataPoint(double velocity, double position, double acceleration, double voltage, long timestamp) {
            this.velocity = velocity;
            this.position = position;
            this.acceleration = acceleration;
            this.rawAcceleration = acceleration;
            this.voltage = voltage;
            this.timestamp = timestamp;
        }
    }

    private class BucketResult {
        double kS, kV, kA, kG, kSin, kV2, avgError, maxError, rSquared;
        int points, rawPoints;

        BucketResult(double kS, double kV, double kA, double kG, double kSin, double kV2, double avgError, double maxError, int points, double rSquared) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kG = kG;
            this.kSin = kSin;
            this.kV2 = kV2;
            this.avgError = avgError;
            this.maxError = maxError;
            this.points = points;
            this.rSquared = rSquared;
        }
    }

    private class KFlags {
        boolean useKS, useKV, useKA, useKG, useKSin, useKV2;
        
        KFlags(boolean useKS, boolean useKV, boolean useKA, boolean useKG, boolean useKSin, boolean useKV2) {
            this.useKS = useKS;
            this.useKV = useKV;
            this.useKA = useKA;
            this.useKG = useKG;
            this.useKSin = useKSin;
            this.useKV2 = useKV2;
        }
    }

    public static void registerMotor(MotorInterface motor) {
        if (!motors.contains(motor)) {
            motors.add(motor);
        }
    }
    
    public static List<MotorInterface> getMotors() {
        return motors;
    }

    public CloseLoopParam getPidParams(String name, List<Entry> motorEntries, boolean[] kFlags) {
        Log.log("Performing analysis...");

        this.name = name;
        this.motorEntries = motorEntries;
        this.kFlags = new KFlags(kFlags[0], kFlags[1], kFlags[2], kFlags[3], kFlags[4], kFlags[5]);;

        BucketResult result = analyzeGroup();

        CloseLoopParam param = new CloseLoopParam();

        if (result == null) {
            return null;
        }

        double kp = 0;

        if (result.kA > 0){
            double kASafe = Math.max(result.kA, 0.01);
        
            kp = result.kV / kASafe;
        }
        
        param.set(
            kp,
            0.0,
            0.0,
            result.kS,
            result.kV,
            result.kA,
            result.kG,
            result.kSin,
            result.kV2
        );

        Log.log(name + " analysis complete.");
        return param;
    }

    private BucketResult analyzeGroup() {
        List<SyncedDataPoint> syncedData = new ArrayList<>();

        if (motorEntries == null || motorEntries.isEmpty()) return null;

        Entry posEntry = null;
        Entry velEntry = null;
        Entry accelEntry = null;
        Entry voltEntry = null;

        for (Entry entry : motorEntries) {
            String entryName = entry.name.toLowerCase();
            if (entryName.contains("pos")) {
                if (posEntry == null) posEntry = entry;
            } else if (entryName.contains("vel")) {
                if (velEntry == null) velEntry = entry;
            } else if (entryName.contains("accel")) {
                if (accelEntry == null) accelEntry = entry;
            } else if (entryName.contains("volt")) {
                if (voltEntry == null) voltEntry = entry;
            }
        }

        if (posEntry == null && motorEntries.size() > 0) posEntry = motorEntries.get(0);
        if (velEntry == null && motorEntries.size() > 1) velEntry = motorEntries.get(1);
        if (accelEntry == null && motorEntries.size() > 2) accelEntry = motorEntries.get(2);
        if (voltEntry == null && motorEntries.size() > 3) voltEntry = motorEntries.get(3);

        if (posEntry != null && velEntry != null && accelEntry != null && voltEntry != null) {
            int minSize = Math.min(Math.min(posEntry.data.size(), velEntry.data.size()),
                                   Math.min(accelEntry.data.size(), voltEntry.data.size()));

            for (int i = 0; i < minSize; i++) {
                Object posVal = posEntry.data.get(i).value;
                Object velVal = velEntry.data.get(i).value;
                Object accelVal = accelEntry.data.get(i).value;
                Object voltVal = voltEntry.data.get(i).value;

                if (posVal instanceof Number && velVal instanceof Number && 
                    accelVal instanceof Number && voltVal instanceof Number) {
                    
                    double pos = ((Number) posVal).doubleValue();
                    double vel = ((Number) velVal).doubleValue();
                    double accel = ((Number) accelVal).doubleValue();
                    double volt = ((Number) voltVal).doubleValue();
                    long time = posEntry.data.get(i).time;

                    syncedData.add(new SyncedDataPoint(vel, pos, accel, volt, time));
                }
            }
        }

        if (syncedData.isEmpty()) {
            for (Entry entry : motorEntries) {
                for (EntryPoint dp : entry.data) {
                    if (dp.value instanceof double[]) {
                        double[] vals = (double[]) dp.value;
                        if (vals.length >= 4) {
                            syncedData.add(new SyncedDataPoint(vals[1], vals[0], vals[2], vals[3], dp.time));
                        }
                    } else if (dp.value instanceof float[]) {
                        float[] vals = (float[]) dp.value;
                        if (vals.length >= 4) {
                            syncedData.add(new SyncedDataPoint(vals[1], vals[0], vals[2], vals[3], dp.time));
                        }
                    }
                }
            }
        }

        System.out.println("Group: " + name + " | Total data points: " + syncedData.size());
        if (syncedData.isEmpty()) return null;

        syncedData.sort((p1, p2) -> Long.compare(p1.timestamp, p2.timestamp));
        
        return calculateResult(syncedData, name);
    }

    private BucketResult calculateResult(List<SyncedDataPoint> rawData, String name) {
        BucketResult model = null;
        double bestAvgError = Double.MAX_VALUE;

        for (double voltageThreshold : VOLTAGE_THRESHOLDS) {
            for (int smoothWindow : SMOOTH_WINDOWS) {
                for (double zScoreThresholds :Z_SCORE_THRESHOLDS) {
                    List<SyncedDataPoint> cleanData = filterAndSmooth(rawData, voltageThreshold, smoothWindow);
                    if (cleanData.size() < 10) continue;

                    BucketResult initialResult = solveOLS(cleanData);
                    if (initialResult == null) continue;

                    List<SyncedDataPoint> refinedData = removeOutliers(cleanData, initialResult, zScoreThresholds);
                    if (refinedData.size() < 10) continue;

                    BucketResult candidateModel = solveOLS(refinedData);

                    double currentKS = kFlags.useKS ? candidateModel.kS : 0;
                    double currentKA = kFlags.useKA ? candidateModel.kA : 0;

                    if (currentKA < 0 || currentKS < 0) {
                        continue;
                    }
                    
                    if (candidateModel != null) {
                        double sumErr = 0;
                        double maxErr = 0;

                        for(SyncedDataPoint p : rawData) {
                            double pred = 0;
                            if(kFlags.useKS) pred += candidateModel.kS * Math.signum(p.velocity);
                            if(kFlags.useKV) pred += candidateModel.kV * p.velocity;
                            if(kFlags.useKA) pred += candidateModel.kA * p.acceleration;
                            if(kFlags.useKG) pred += candidateModel.kG * 1.0;
                            if(kFlags.useKSin) pred += candidateModel.kSin * Math.cos(p.position);
                            if(kFlags.useKV2) pred += candidateModel.kV2 * p.velocity * Math.abs(p.velocity);
                            
                            double error = Math.abs(p.voltage - pred);
                            sumErr += error;
                            if(error > maxErr) maxErr = error;
                        }

                        candidateModel.avgError = sumErr / rawData.size();
                        candidateModel.maxError = maxErr;
                        candidateModel.rawPoints = rawData.size();

                        if (candidateModel.avgError < bestAvgError) {
                            bestAvgError = candidateModel.avgError;
                            model = candidateModel;
                        }
                    }
                }
            }
        }
        
        if (model == null) {
            return null;
        }
        
        double sumErr = 0;
        double maxErr = 0;

        for(SyncedDataPoint p : rawData) {
            double pred = 0;
            if(kFlags.useKS) pred += model.kS * Math.signum(p.velocity);
            if(kFlags.useKV) pred += model.kV * p.velocity;
            if(kFlags.useKA) pred += model.kA * p.acceleration;
            if(kFlags.useKG) pred += model.kG * 1.0;
            if(kFlags.useKSin) pred += model.kSin * Math.cos(p.position);
            if(kFlags.useKV2) pred += model.kV2 * p.velocity * Math.abs(p.velocity);
            
            double error = Math.abs(p.voltage - pred);
            sumErr += error;
            if(error > maxErr) maxErr = error;
        }

        model.avgError = sumErr / rawData.size();
        model.maxError = maxErr;
        model.rawPoints = rawData.size();

        Log.log(name + " avg Error: " + model.avgError);
        Log.log(name + " max Error: " + model.maxError);
        Log.log(name + " used Points size: " + model.points);
        Log.log(name + " raw Points size: " + model.rawPoints);
        Log.log(name + " r Squared: " + model.rSquared);
    
        return model;
    }

    private List<SyncedDataPoint> filterAndSmooth(List<SyncedDataPoint> rawData, double voltageThresh, int windowSize) {
        List<SyncedDataPoint> filtered = new ArrayList<>();
        for (int i = 0; i < rawData.size(); i++) {
            SyncedDataPoint current = rawData.get(i);
            
            double sumAccel = 0;
            int count = 0;
            for (int j = Math.max(0, i - windowSize/2); j < Math.min(rawData.size(), i + windowSize/2 + 1); j++) {
                sumAccel += rawData.get(j).rawAcceleration;
                count++;
            }
            current.acceleration = sumAccel / count;
            
            if (Math.abs(current.voltage) > voltageThresh) {
                filtered.add(current);
            }
        }
        return filtered;
    }

    private List<SyncedDataPoint> removeOutliers(List<SyncedDataPoint> data, BucketResult model, double zScoreThreshold) {
        if (zScoreThreshold <= 0) return data;

        double kS = model.kS;
        double kV = model.kV;
        double kA = model.kA;
        double kG = model.kG;
        double kCos = model.kSin;
        double kV2 = model.kV2;

        double sumError = 0;

        for (SyncedDataPoint p : data) {
            double pred = 0;
            if(kFlags.useKS) pred += kS * Math.signum(p.velocity);
            if(kFlags.useKV) pred += kV * p.velocity;
            if(kFlags.useKA) pred += kA * p.acceleration;
            if(kFlags.useKG) pred += kG * 1.0;
            if(kFlags.useKSin) pred += kCos * Math.cos(p.position);
            if(kFlags.useKV2) pred += kV2 * p.velocity * Math.abs(p.velocity);
            
            p.error = Math.abs(p.voltage - pred);
            sumError += p.error;
        }

        double meanError = sumError / data.size();
        double sumSqDiff = 0;

        for (SyncedDataPoint p : data) {
            sumSqDiff += Math.pow(p.error - meanError, 2);
        }
        double stdDev = Math.sqrt(sumSqDiff / data.size());

        List<SyncedDataPoint> filteredData = new ArrayList<>();
        double maxAllowedError = meanError + (zScoreThreshold * stdDev);

        for (SyncedDataPoint p : data) {
            if (p.error <= maxAllowedError) {
                filteredData.add(p);
            }
        }

        return filteredData;
    }

    private BucketResult solveOLS(List<SyncedDataPoint> data) {
        int n = data.size();
        int numParams = 0;
        if(kFlags.useKS) numParams++;
        if(kFlags.useKV) numParams++;
        if(kFlags.useKA) numParams++;
        if(kFlags.useKG) numParams++;
        if(kFlags.useKSin) numParams++;
        if(kFlags.useKV2) numParams++;

        if(numParams == 0) return null;

        SimpleMatrix A = new SimpleMatrix(n, numParams);
        SimpleMatrix b = new SimpleMatrix(n, 1);

        for (int i = 0; i < n; i++) {
            SyncedDataPoint p = data.get(i);
            b.set(i, 0, p.voltage);

            int col = 0;
            if(kFlags.useKS) A.set(i, col++, Math.signum(p.velocity));
            if(kFlags.useKV) A.set(i, col++, p.velocity);
            if(kFlags.useKA) A.set(i, col++, p.acceleration);
            if(kFlags.useKG) A.set(i, col++, 1.0);
            if(kFlags.useKSin) A.set(i, col++, Math.cos(p.position));
            if(kFlags.useKV2) A.set(i, col++, p.velocity * Math.abs(p.velocity));
        }

        SimpleMatrix x;
        try {
            x = A.solve(b);
        } catch(Exception e) {
            return null;
        }

        double[] k = new double[6];
        int col = 0;
        if(kFlags.useKS) k[0] = x.get(col++);
        if(kFlags.useKV) k[1] = x.get(col++);
        if(kFlags.useKA) k[2] = x.get(col++);
        if(kFlags.useKG) k[3] = x.get(col++);
        if(kFlags.useKSin) k[4] = x.get(col++);
        if(kFlags.useKV2) k[5] = x.get(col++);

        double ssTot = 0, ssRes = 0, meanV = 0;
        for(SyncedDataPoint p : data) meanV += p.voltage;
        meanV /= n;

        for (SyncedDataPoint p : data) {
            double pred = 0;
            if(kFlags.useKS) pred += k[0] * Math.signum(p.velocity);
            if(kFlags.useKV) pred += k[1] * p.velocity;
            if(kFlags.useKA) pred += k[2] * p.acceleration;
            if(kFlags.useKG) pred += k[3] * 1.0;
            if(kFlags.useKSin) pred += k[4] * Math.cos(p.position);
            if(kFlags.useKV2) pred += k[5] * p.velocity * Math.abs(p.velocity);

            ssTot += Math.pow(p.voltage - meanV, 2);
            ssRes += Math.pow(p.voltage - pred, 2);
        }

        double r2 = 1 - (ssRes / ssTot);

        return new BucketResult(k[0], k[1], k[2], k[3], k[4], k[5], 0, 0, n, r2);
    }
}
