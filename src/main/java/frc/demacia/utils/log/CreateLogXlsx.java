package frc.demacia.utils.log;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.Color;
import java.awt.Font;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.*;
import java.util.List;
import org.apache.poi.ss.usermodel.*;
import org.apache.poi.xssf.usermodel.XSSFWorkbook;

public class CreateLogXlsx {
    public static void main(String[] args) {
        try {
            UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
        } catch (Exception e) {
            e.printStackTrace();
        }

        SwingUtilities.invokeLater(() -> {
            CreateLogFrame app = new CreateLogFrame();
            app.setVisible(true);
        });
    }
}

class CreateLogFrame extends JFrame {
    private JTextField filePathField;
    private JList<String> groupListModel;
    private DefaultListModel<String> listModel = new DefaultListModel<>();
    private JTextArea msgArea = new JTextArea();
    private JButton exportButton;

    private Map<String, List<LogReader.Entry>> loadedGroups = null;
    private static CreateLogFrame instance = null;

    public CreateLogFrame() {
        super("WPILog to Excel Exporter (.xlsx)");
        instance = this;

        setSize(1100, 750);
        setLocationRelativeTo(null);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        setLayout(new GridBagLayout());
        GridBagConstraints gbc = new GridBagConstraints();

        JPanel topPanel = new JPanel(new BorderLayout(10, 0));
        topPanel.setBorder(BorderFactory.createTitledBorder("Log Source & Action"));

        filePathField = new JTextField();
        filePathField.setEditable(false);
        filePathField.setBackground(Color.WHITE);

        JPanel buttonPanel = new JPanel(new FlowLayout(FlowLayout.RIGHT, 5, 0));
        JButton openFileBtn = new JButton("Open WPILOG");
        exportButton = new JButton("Export to Excel (.xlsx)");
        exportButton.setEnabled(false);

        openFileBtn.addActionListener(e -> loadLog());
        exportButton.addActionListener(e -> exportToExcel());

        buttonPanel.add(openFileBtn);
        buttonPanel.add(exportButton);

        topPanel.add(filePathField, BorderLayout.CENTER);
        topPanel.add(buttonPanel, BorderLayout.EAST);

        gbc.gridx = 0; gbc.gridy = 0;
        gbc.weightx = 1.0; gbc.weighty = 0.0;
        gbc.fill = GridBagConstraints.HORIZONTAL;
        gbc.insets = new Insets(10, 10, 5, 10);
        add(topPanel, gbc);

        groupListModel = new JList<>(listModel);
        groupListModel.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
        groupListModel.setBorder(BorderFactory.createTitledBorder("Loaded Log Groups"));
        JScrollPane listScroll = new JScrollPane(groupListModel);

        gbc.gridx = 0; gbc.gridy = 1;
        gbc.weightx = 1.0; gbc.weighty = 1.0;
        gbc.fill = GridBagConstraints.BOTH;
        gbc.insets = new Insets(5, 10, 5, 10);
        add(listScroll, gbc);

        msgArea.setEditable(false);
        msgArea.setFont(new Font("Consolas", Font.PLAIN, 12));
        JScrollPane msgPane = new JScrollPane(msgArea, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_ALWAYS);
        msgPane.setBorder(BorderFactory.createTitledBorder("Console"));
        msgPane.setPreferredSize(new Dimension(100, 150));

        gbc.gridx = 0; gbc.gridy = 2;
        gbc.weightx = 1.0; gbc.weighty = 0.25;
        gbc.fill = GridBagConstraints.BOTH;
        gbc.insets = new Insets(5, 10, 10, 10);
        add(msgPane, gbc);

        msg("Ready. Click 'Open WPILOG' to begin.");
    }

    private void loadLog() {
        new Thread(() -> {
            try {
                msg("Opening file explorer...");
                loadedGroups = LogReader.getGroups(false, null);

                SwingUtilities.invokeLater(() -> {
                    listModel.clear();
                    if (loadedGroups != null && !loadedGroups.isEmpty()) {
                        List<String> sortedKeys = new ArrayList<>(loadedGroups.keySet());
                        Collections.sort(sortedKeys);
                        for (String key : sortedKeys) {
                            listModel.addElement(key + " (" + loadedGroups.get(key).size() + " entries)");
                        }
                        filePathField.setText("WPILOG File Loaded Successfully");
                        exportButton.setEnabled(true);
                        msg("Successfully loaded " + loadedGroups.size() + " log groups.");
                    } else {
                        filePathField.setText("No groups found");
                        exportButton.setEnabled(false);
                        msg("Warning: No log groups found in the file.");
                    }
                });
            } catch (Exception e) {
                SwingUtilities.invokeLater(() -> {
                    msg("Error loading log file: " + e.getMessage());
                    filePathField.setText("Error loading file");
                    exportButton.setEnabled(false);
                });
            }
        }).start();
    }

    private void exportToExcel() {
        if (loadedGroups == null || loadedGroups.isEmpty()) {
            msg("No data available to export.");
            return;
        }

        try {
            String savePath = selectSaveFileFromComputer("RobotLogExport.xlsx");
            
            if (savePath != null && !savePath.trim().isEmpty()) {
                if (!savePath.toLowerCase().endsWith(".xlsx")) {
                    savePath += ".xlsx";
                }

                final String finalPath = savePath;
                new Thread(() -> {
                    try {
                        msg("Generating Excel file at: " + finalPath + " ...");
                        createXlsxFile(finalPath, loadedGroups);
                        msg("Excel file successfully created and saved!");
                        SwingUtilities.invokeLater(() -> 
                            JOptionPane.showMessageDialog(this, "Excel file exported successfully!\n" + finalPath, "Success", JOptionPane.INFORMATION_MESSAGE)
                        );
                    } catch (Exception e) {
                        msg("Error generating Excel file: " + e.getMessage());
                        e.printStackTrace();
                        SwingUtilities.invokeLater(() -> 
                            JOptionPane.showMessageDialog(this, "Error exporting Excel: " + e.getMessage(), "Error", JOptionPane.ERROR_MESSAGE)
                        );
                    }
                }).start();
            }
        } catch (Exception e) {
            msg("Export cancelled or failed: " + e.getMessage());
        }
    }

    private String selectSaveFileFromComputer(String defaultFileName) throws IOException {
        if (GraphicsEnvironment.isHeadless()) {
            throw new UnsupportedOperationException("Cannot open GUI file picker in a headless environment.");
        }

        String userHome = System.getProperty("user.home");
        String initialPath = userHome + "/Downloads";
        File initialDir = new File(initialPath);
        String escapedPath = initialDir.exists() ? initialDir.getAbsolutePath().replace("\\", "\\\\") : "";

        String psScript = "$showDialog = New-Object System.Windows.Forms.SaveFileDialog;" +
                          "$showDialog.InitialDirectory = '" + escapedPath + "';" +
                          "$showDialog.FileName = '" + defaultFileName + "';" +
                          "$showDialog.Filter = 'Excel Workbooks (*.xlsx)|*.xlsx';" +
                          "$showDialog.Title = 'Save Excel File';" +
                          "$showDialog.OverwritePrompt = $true;" +
                          "$result = $showDialog.ShowDialog();" +
                          "if ($result -eq 'OK') { Write-Output $showDialog.FileName }";

        try {
            ProcessBuilder pb = new ProcessBuilder(
                "powershell", 
                "-NoProfile", 
                "-Command", 
                "[void][System.Reflection.Assembly]::LoadWithPartialName('System.Windows.Forms'); " + psScript
            );
            Process process = pb.start();

            BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            String selectedPath = reader.readLine();
            process.waitFor();

            if (selectedPath != null && !selectedPath.trim().isEmpty()) {
                return selectedPath.trim();
            } else {
                throw new IOException("Excel file save was cancelled by the user.");
            }
        } catch (Exception e) {
            return fallbackSaveFileChooser(defaultFileName);
        }
    }

    private String fallbackSaveFileChooser(String defaultFileName) throws IOException {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setDialogTitle("Save Excel File");
        fileChooser.setSelectedFile(new File(defaultFileName));
        fileChooser.setFileFilter(new FileNameExtensionFilter("Excel Workbooks (*.xlsx)", "xlsx"));
        fileChooser.setPreferredSize(new Dimension(1000, 650));

        int userSelection = fileChooser.showSaveDialog(this);
        if (userSelection == JFileChooser.APPROVE_OPTION) {
            return fileChooser.getSelectedFile().getAbsolutePath();
        } else {
            throw new IOException("Excel file save was cancelled by the user.");
        }
    }

    private void createXlsxFile(String filePath, Map<String, List<LogReader.Entry>> groups) throws IOException {
        try (Workbook workbook = new XSSFWorkbook()) {
            Set<String> usedSheetNames = new HashSet<>();

            for (Map.Entry<String, List<LogReader.Entry>> groupEntry : groups.entrySet()) {
                String groupName = groupEntry.getKey();
                List<LogReader.Entry> entries = groupEntry.getValue();

                String sheetName = sanitizeAndUniqueSheetName(groupName.isEmpty() ? "General" : groupName, usedSheetNames);
                Sheet sheet = workbook.createSheet(sheetName);

                Row headerRow = sheet.createRow(0);
                headerRow.createCell(0).setCellValue("Timestamp (us)");
                
                int colIndex = 1;
                Map<String, Integer> entryColMap = new HashMap<>();
                for (LogReader.Entry entry : entries) {
                    headerRow.createCell(colIndex).setCellValue(entry.name);
                    entryColMap.put(entry.name, colIndex);
                    colIndex++;
                }

                Map<Long, Map<String, Object>> timeDataMap = new TreeMap<>();
                for (LogReader.Entry entry : entries) {
                    for (LogReader.EntryPoint point : entry.data) {
                        timeDataMap.computeIfAbsent(point.time, k -> new HashMap<>()).put(entry.name, point.value);
                    }
                }

                int rowIndex = 1;
                for (Map.Entry<Long, Map<String, Object>> rowEntry : timeDataMap.entrySet()) {
                    long time = rowEntry.getKey();
                    Map<String, Object> values = rowEntry.getValue();

                    Row row = sheet.createRow(rowIndex++);
                    row.createCell(0).setCellValue(time);

                    for (Map.Entry<String, Object> valEntry : values.entrySet()) {
                        String entryName = valEntry.getKey();
                        Object val = valEntry.getValue();
                        Integer cIdx = entryColMap.get(entryName);
                        if (cIdx != null) {
                            Cell cell = row.createCell(cIdx);
                            if (val instanceof Number) {
                                cell.setCellValue(((Number) val).doubleValue());
                            } else if (val instanceof Boolean) {
                                cell.setCellValue((Boolean) val);
                            } else if (val != null) {
                                cell.setCellValue(val.toString());
                            }
                        }
                    }
                }

                for (int i = 0; i < colIndex; i++) {
                    sheet.autoSizeColumn(i);
                }
            }

            try (FileOutputStream fos = new FileOutputStream(filePath)) {
                workbook.write(fos);
            }
        }
    }

    private String sanitizeAndUniqueSheetName(String originalName, Set<String> usedNames) {
        if (originalName == null || originalName.isEmpty()) {
            originalName = "Sheet";
        }
        String sanitized = originalName.replaceAll("[\\\\/?*+\\[\\]:]", "_");
        
        if (sanitized.length() > 31) {
            sanitized = sanitized.substring(0, 31);
        }
        
        String finalName = sanitized;
        int counter = 1;
        while (usedNames.contains(finalName)) {
            String suffix = "_" + counter;
            int maxBaseLength = 31 - suffix.length();
            String base = sanitized.length() > maxBaseLength ? sanitized.substring(0, maxBaseLength) : sanitized;
            finalName = base + suffix;
            counter++;
        }
        usedNames.add(finalName);
        return finalName;
    }

    public static void msg(String message) {
        if (instance != null) {
            SwingUtilities.invokeLater(() -> {
                instance.msgSpaceAppender(message);
            });
        }
    }

    private void msgSpaceAppender(String message) {
        msgArea.append(message + "\n");
        msgArea.setCaretPosition(msgArea.getDocument().getLength());
    }
}