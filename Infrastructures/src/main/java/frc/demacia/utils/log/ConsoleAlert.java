package frc.demacia.utils.log;

import edu.wpi.first.wpilibj.Alert;

/** STAND-IN for the team's ConsoleAlert (subset). Delete when moved into DemaciaCode. */
public class ConsoleAlert extends Alert {
    public static final String DEFAULT_GROUP = "console";

    public ConsoleAlert(String text, AlertType type) {
        super(DEFAULT_GROUP, text, type);
    }

    public static ConsoleAlert warning(String text) {
        return new ConsoleAlert(text, AlertType.kWarning);
    }

    public static ConsoleAlert error(String text) {
        return new ConsoleAlert(text, AlertType.kError);
    }
}
