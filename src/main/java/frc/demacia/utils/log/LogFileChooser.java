package frc.demacia.utils.log;

import java.awt.GraphicsEnvironment;
import java.io.File;
import java.io.IOException;
import javax.swing.JFileChooser;
import javax.swing.UIManager;
import javax.swing.filechooser.FileNameExtensionFilter;

public class LogFileChooser {

    /**
     * Opens a native OS file dialog to select a .wpilog file.
     * Automatically sets the starting directory to where FRC tools download log files.
     * 
     * @return Absolute path of the selected .wpilog file
     * @throws IOException If the user cancels or if running in a headless environment
     */
    public static String selectFileFromComputer() throws IOException {
        if (GraphicsEnvironment.isHeadless()) {
            throw new UnsupportedOperationException("Cannot open GUI file picker in a headless environment (e.g., on the roboRIO).");
        }

        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception ignored) {
        }

        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setDialogTitle("Select WPILog File");
        fileChooser.setMultiSelectionEnabled(false);

        FileNameExtensionFilter filter = new FileNameExtensionFilter("WPILib Log Files (*.wpilog)", "wpilog");
        fileChooser.setFileFilter(filter);
        fileChooser.setAcceptAllFileFilterUsed(false);

        File initialDirectory = getBestLogDirectory();
        if (initialDirectory != null && initialDirectory.exists()) {
            fileChooser.setCurrentDirectory(initialDirectory);
        }

        int userSelection = fileChooser.showOpenDialog(null);

        if (userSelection == JFileChooser.APPROVE_OPTION) {
            File selectedFile = fileChooser.getSelectedFile();
            return selectedFile.getAbsolutePath();
        } else {
            throw new IOException("Log file selection was cancelled by the user.");
        }
    }

    /**
     * Finds the best default folder where logs are usually downloaded.
     */
    private static File getBestLogDirectory() {
        String userHome = System.getProperty("user.home");
        String publicDir = System.getenv("PUBLIC");

        File[] potentialDirs = new File[] {
            new File(publicDir != null ? publicDir + "/Documents/FRC/Log Files" : "C:/Users/Public/Documents/FRC/Log Files"),
            new File("logs/"),
            new File(userHome + "/Documents"),
            new File(userHome + "/Downloads"),
        };

        for (File dir : potentialDirs) {
            if (dir.exists() && dir.isDirectory()) {
                File[] wpilogFiles = dir.listFiles((d, name) -> name.endsWith(".wpilog"));
                if (wpilogFiles != null && wpilogFiles.length > 0) {
                    return dir;
                }
            }
        }

        for (File dir : potentialDirs) {
            if (dir.exists() && dir.isDirectory()) {
                return dir;
            }
        }

        return null;
    }
}