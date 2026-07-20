package frc.demacia.utils.log;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LogReplyCommand extends InstantCommand {
    public LogReplyCommand() {
        super(() -> {
            LogReply.loadFile();
        });
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}