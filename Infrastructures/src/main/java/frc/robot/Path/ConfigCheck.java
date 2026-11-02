package frc.robot.Path;

/** Shared range check of the config records (design doc section 9.12, V16). */
final class ConfigCheck {

    private ConfigCheck() {
    }

    /**
     * Throws unless value is a finite number above 0.
     *
     * @param record  record name, for example "PathConstraints"
     * @param field   field name, for example "vMax"
     * @param value   the value to check
     * @param unit    its unit, for example "m/s"
     * @param factory the factory to start from, for example "PathConstraints.fromConfig(config)"
     * @param example a complete example call, for example "PathConstraints.fromConfig(config).withVMax(3.0)"
     */
    static void requirePositive(String record, String field, double value, String unit, String factory,
                                String example) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(message(record, field, "above 0", unit, value, factory, example));
        }
    }

    /** Same as {@link #requirePositive} but 0 is allowed. */
    static void requireNotNegative(String record, String field, double value, String unit, String factory,
                                   String example) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(message(record, field, "0 or more", unit, value, factory, example));
        }
    }

    private static String message(String record, String field, String rule, String unit, double value,
                                  String factory, String example) {
        return record + "." + field + " must be a finite number " + rule + " (" + unit + "), but it is " + value
                + ". Start from " + factory + " and change one value, for example " + example + ".";
    }
}
