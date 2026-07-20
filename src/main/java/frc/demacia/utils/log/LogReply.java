package frc.demacia.utils.log;

import java.io.DataInputStream;
import java.io.EOFException;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.demacia.sysID.LogReader;

public class LogReply implements Sendable{

    private static LogReply instance;

    private long time;
    public static class Group {
        public String rewGroupName;
        public String groupName;
        public String type;
        public String metaData;
        public Map<String, List<DataPoint>> data = new HashMap<>();

        public Group(String rewGroupName, String type, String metaData) {
            this.rewGroupName = rewGroupName;
            this.type = type;
            this.metaData = metaData;

            String[] parts = rewGroupName.split(": ");
            if (parts.length > 1) {
                this.groupName = parts[0];
                String[] keys = parts[1].split(", ");
                for (String key : keys) {
                    data.put(key, new ArrayList<>());
                }
            } else {
                this.groupName = "";
                data.put(rewGroupName, new ArrayList<>());
            }
        }
    }

    public static class DataPoint {
        public long timestamp;
        public double values;

        public DataPoint(long timestamp, double values) {
            this.timestamp = timestamp;
            this.values = values;
        }
    }

    private static class ReplayEntry {
        String path;
        List<DataPoint> points;
    
        int currentIndex = 0;
        double currentValue = 0;
    
        ReplayEntry(String path, List<DataPoint> points) {
            this.path = path;
            this.points = points;
    
            if (!points.isEmpty()) {
                currentValue = points.get(0).values;
            }
        }
    
        public double getValue() {
            return currentValue;
        }
    
        public void update(long time) {
    
            if (points.isEmpty())
                return;
    
            while (currentIndex > 0 &&
                    points.get(currentIndex).timestamp > time) {
                currentIndex--;
            }
    
            while (currentIndex < points.size() - 1 &&
                    points.get(currentIndex + 1).timestamp <= time) {
                currentIndex++;
            }
    
            currentValue = points.get(currentIndex).values;
        }
    }

    private List<ReplayEntry> replayEntries = new ArrayList<>();

    public static Map<Integer, List<Group>> entries = new HashMap<>();

    @SuppressWarnings("unchecked")
    public static void loadFile() {
        entries.clear();
        try {
            System.out.println("Reading log file: ");
            
            loadLatestRobotLog();

            instance = new LogReply();

            for (List<Group> groups : entries.values()) {
                for (Group group : groups) {
                    for (Map.Entry<String, List<DataPoint>> entry : group.data.entrySet()) {
                        ReplayEntry replay = new ReplayEntry(
                                "reUpload/" + group.groupName + "/" + entry.getKey(),
                                entry.getValue());

                                instance.replayEntries.add(replay);

                        Log.putData(
                                replay.path,
                                replay::getValue);
                    }
                }
            }

            CommandScheduler.getInstance().schedule(
                Commands.runOnce(() -> 
                    SmartDashboard.putData("LogReplayControl", instance)
                )
            );
        } catch (IOException e) {
            System.err.println("Error reading log file: " + e.getMessage());
            e.printStackTrace();
        }
    }

    public static void loadLatestRobotLog() throws IOException {
        DataLogManager.getLog().flush();

        File latestLogFile = getLatestLogFile();
        if (latestLogFile == null) {
            throw new IOException("SysID Error: No .wpilog files found on the robot!");
        }

        Log.log("SysID: Successfully found and loading latest log: " + latestLogFile.getAbsolutePath());

        wpilogReader(latestLogFile.getAbsolutePath()); 
    }

    private static File getLatestLogFile() {
        File logDir;

        if (RobotBase.isSimulation()) {
            logDir = new File("logs/");
        } else {
            logDir = new File("/home/lvuser/logs/");
        }

        if (!logDir.exists()) {
            return null;
        }
        
        File[] files = logDir.listFiles((dir, name) -> name.endsWith(".wpilog"));

        if (files == null || files.length == 0) {
            return null;
        }

        File latestFile = files[0];
        for (File file : files) {
            if (file.lastModified() > latestFile.lastModified()) {
                latestFile = file;
            }
        }
        return latestFile;
    }

    private static void wpilogReader(String fileName) throws IOException {
        try (FileInputStream fileInputStream = new FileInputStream(fileName);
             DataInputStream dataInputStream = new DataInputStream(fileInputStream)) {

            byte[] signature = readHeader(dataInputStream);
            if (!Arrays.equals(signature, "WPILOG".getBytes())) {
                throw new IOException("Invalid WPILOG file format. Expected WPILOG, got: " + new String(signature));
            }

            skipHeaderExtra(dataInputStream);
            readRecords(dataInputStream);

            
        }
    }

    private static byte[] readHeader(DataInputStream dataInputStream) throws IOException {
        byte[] signature = new byte[6];
        dataInputStream.readFully(signature);
        return signature;
    }

    private static void skipHeaderExtra(DataInputStream dataInputStream) throws IOException {
        short version = Short.reverseBytes(dataInputStream.readShort());
        int extraLength = Integer.reverseBytes(dataInputStream.readInt());
        System.out.println("WPILOG version: " + version + ", extra header length: " + extraLength);
        if (extraLength > 0) {
            dataInputStream.skipBytes(extraLength);
        }
    }

    private static void readRecords(DataInputStream dataInputStream) throws IOException {
        int recordCount = 0;
        int dataRecordsProcessed = 0;

        int n = 0;
        while (true) {
            try {
                n++;
                if(n%1000 == 0) {
                    Log.log("Read " + n + " records...");
                }
                if (readRecord(dataInputStream)) {
                    dataRecordsProcessed++;
                }
                recordCount++;
            } catch (EOFException e) {
                break;
            }
        }
        System.out.println("Total records scanned: " + recordCount);
        System.out.println("Valid data records stored: " + dataRecordsProcessed);
    }

    private static boolean readRecord(DataInputStream dataInputStream) throws IOException {
        int headerByte = dataInputStream.readUnsignedByte();
        int idLength = (headerByte & 0x3) + 1;
        int payloadLength = (headerByte >> 2 & 0x3) + 1;
        int timestampLength = (headerByte >> 4 & 0x7) + 1;

        int recordId = readLittleEndianInt(dataInputStream, idLength);
        int payloadSize = readLittleEndianInt(dataInputStream, payloadLength);
        long timestamp = readLittleEndianLong(dataInputStream, timestampLength);

        if (recordId == 0) {
            addEntryFromControlRecord(dataInputStream, payloadSize);
            return false;
        } else {
            List<Group> groups = entries.get(recordId);

            if (groups == null || groups.isEmpty()) {
                dataInputStream.skipBytes(payloadSize);
                return false;
            }

            String type = groups.get(0).type;
            double[] values = null;

            if (type.equals("double") || type.equals("double[]")) {
                if (payloadSize % 8 == 0) {
                    int count = payloadSize / 8;
                    values = new double[count];
                    for (int i = 0; i < count; i++) {
                        values[i] = Double.longBitsToDouble(Long.reverseBytes(dataInputStream.readLong()));
                    }
                } else { dataInputStream.skipBytes(payloadSize); }
                
            } else if (type.equals("float") || type.equals("float[]")) {
                if (payloadSize % 4 == 0) {
                    int count = payloadSize / 4;
                    values = new double[count];
                    for (int i = 0; i < count; i++) {
                        values[i] = (double) Float.intBitsToFloat(Integer.reverseBytes(dataInputStream.readInt()));
                    }
                } else { dataInputStream.skipBytes(payloadSize); }

            } else if (type.equals("boolean") || type.equals("boolean[]")) {
                int count = payloadSize; 
                values = new double[count];
                for (int i = 0; i < count; i++) {
                    values[i] = (dataInputStream.readByte() != 0) ? 1.0 : 0.0;
                }

            } else if (type.equals("int64") || type.equals("int64[]")) {
                if (payloadSize % 8 == 0) {
                    int count = payloadSize / 8;
                    values = new double[count];
                    for (int i = 0; i < count; i++) {
                        values[i] = (double) Long.reverseBytes(dataInputStream.readLong());
                    }
                } else { dataInputStream.skipBytes(payloadSize); }
                
            } else {
                dataInputStream.skipBytes(payloadSize);
            }

            if (values != null) {
                int totalFields = 0;
                for (Group g : groups) {
                    String[] parts = g.rewGroupName.split(": ");
                    if (parts.length > 1) {
                        totalFields += parts[1].split(", ").length;
                    } else {
                        totalFields += 1;
                    }
                }

                if (totalFields > 0 && values.length % totalFields == 0) {
                    int samples = values.length / totalFields;
                    for (int s = 0; s < samples; s++) {
                        int currentIndex = s * totalFields;
                        for (Group g : groups) {
                            String[] parts = g.rewGroupName.split(": ");
                            String[] fieldNames;
                            if (parts.length > 1) {
                                fieldNames = parts[1].split(", ");
                            } else {
                                fieldNames = new String[]{g.rewGroupName};
                            }

                            for (String fieldName : fieldNames) {
                                double val = values[currentIndex++];
                                g.data.get(fieldName).add(new DataPoint(timestamp, val));
                            }
                        }
                    }
                    return true;
                }
            }
            return false;
        }
    }

    private static void addEntryFromControlRecord(DataInputStream dataInputStream, int payloadSize) throws IOException {
        int recordType = dataInputStream.readUnsignedByte();
        if (recordType == 0) {
            int entryId = Integer.reverseBytes(dataInputStream.readInt());
            int nameLength = Integer.reverseBytes(dataInputStream.readInt());
            String name = readString(dataInputStream, nameLength);
            int typeLength = Integer.reverseBytes(dataInputStream.readInt());
            String type = readString(dataInputStream, typeLength);
            int metaLength = Integer.reverseBytes(dataInputStream.readInt());
            String metadata = readString(dataInputStream, metaLength);

            String[] names = name.split(" \\| ");
            String[] metas = metadata.split(" \\| ");

            for (int i = 0; i < names.length; i++) {
                String currentName = names[i].trim();
                String currentMeta = (i < metas.length) ? metas[i].trim() : "";
                entries.putIfAbsent(entryId, new ArrayList<>());
                entries.get(entryId).add(new Group(currentName, type, currentMeta));
            }
        } else {
            if (payloadSize > 1) {
                dataInputStream.skipBytes(payloadSize - 1);
            }
        }
    }

    private static String readString(DataInputStream dataInputStream, int length) throws IOException {
        byte[] bytes = new byte[length];
        dataInputStream.readFully(bytes);
        return new String(bytes, "UTF-8");
    }

    private static int readLittleEndianInt(DataInputStream dis, int bytes) throws IOException {
        int result = 0;
        for (int i = 0; i < bytes; i++) {
            result |= (dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    private static long readLittleEndianLong(DataInputStream dis, int bytes) throws IOException {
        long result = 0L;
        for (int i = 0; i < bytes; i++) {
            result |= ((long) dis.readUnsignedByte() << (i * 8));
        }
        return result;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addIntegerProperty("time", () -> time, (time) -> {
            this.time = time;

            for (ReplayEntry replay : replayEntries) {
                replay.update(time);
            }
        });
    }
        
    
}