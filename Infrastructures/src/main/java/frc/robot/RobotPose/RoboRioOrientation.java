package frc.robot.RobotPose;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * How the roboRIO is mounted on the robot, so its built-in accelerometer can be read in the
 * robot's frame (x forward, y left). Used by {@link RobotPose} for collision detection.
 *
 * <p>Each value is named by where the roboRIO's <b>label</b> faces (the accelerometer's +Z axis
 * comes out of the label: flat with the label up, Z reads +1 g) and where the accelerometer's
 * <b>X axis</b> points (drawn on the roboRIO). For example {@link #LABEL_LEFT_X_FRONT}: the
 * roboRIO stands on its side with the label facing the robot's left, and its X axis points to
 * the robot's front.
 *
 * <p>To check the choice on the robot (disabled, on the floor): {@code pose/acceleration x g} and
 * {@code y g} read about 0 at rest (if one reads about +-1 g, the X direction is wrong), lifting
 * the robot's front makes {@code x g} positive, and lifting its left side makes {@code y g}
 * positive.
 */
public enum RoboRioOrientation {
    LABEL_UP_X_FRONT(Direction.UP, Direction.FRONT),
    LABEL_UP_X_BACK(Direction.UP, Direction.BACK),
    LABEL_UP_X_LEFT(Direction.UP, Direction.LEFT),
    LABEL_UP_X_RIGHT(Direction.UP, Direction.RIGHT),

    LABEL_DOWN_X_FRONT(Direction.DOWN, Direction.FRONT),
    LABEL_DOWN_X_BACK(Direction.DOWN, Direction.BACK),
    LABEL_DOWN_X_LEFT(Direction.DOWN, Direction.LEFT),
    LABEL_DOWN_X_RIGHT(Direction.DOWN, Direction.RIGHT),

    LABEL_LEFT_X_FRONT(Direction.LEFT, Direction.FRONT),
    LABEL_LEFT_X_BACK(Direction.LEFT, Direction.BACK),
    LABEL_LEFT_X_UP(Direction.LEFT, Direction.UP),
    LABEL_LEFT_X_DOWN(Direction.LEFT, Direction.DOWN),

    LABEL_RIGHT_X_FRONT(Direction.RIGHT, Direction.FRONT),
    LABEL_RIGHT_X_BACK(Direction.RIGHT, Direction.BACK),
    LABEL_RIGHT_X_UP(Direction.RIGHT, Direction.UP),
    LABEL_RIGHT_X_DOWN(Direction.RIGHT, Direction.DOWN),

    LABEL_FRONT_X_LEFT(Direction.FRONT, Direction.LEFT),
    LABEL_FRONT_X_RIGHT(Direction.FRONT, Direction.RIGHT),
    LABEL_FRONT_X_UP(Direction.FRONT, Direction.UP),
    LABEL_FRONT_X_DOWN(Direction.FRONT, Direction.DOWN),

    LABEL_BACK_X_LEFT(Direction.BACK, Direction.LEFT),
    LABEL_BACK_X_RIGHT(Direction.BACK, Direction.RIGHT),
    LABEL_BACK_X_UP(Direction.BACK, Direction.UP),
    LABEL_BACK_X_DOWN(Direction.BACK, Direction.DOWN);

    /** Robot-frame unit vectors (forward, left, up) of the roboRIO's X, Y and Z axes. */
    private final double[] xAxis;
    private final double[] yAxis;
    private final double[] zAxis;

    RoboRioOrientation(Direction label, Direction x) {
        zAxis = label.vector;
        xAxis = x.vector;
        // Right-handed axes: X x Y = Z, so Y = Z x X.
        yAxis = new double[] {
                zAxis[1] * xAxis[2] - zAxis[2] * xAxis[1],
                zAxis[2] * xAxis[0] - zAxis[0] * xAxis[2],
                zAxis[0] * xAxis[1] - zAxis[1] * xAxis[0] };
    }

    /**
     * Turns a roboRIO accelerometer reading into the robot's horizontal plane.
     *
     * @return (forward, left) in the same units as the reading.
     */
    public Translation2d toRobotHorizontal(double x, double y, double z) {
        return new Translation2d(
                x * xAxis[0] + y * yAxis[0] + z * zAxis[0],
                x * xAxis[1] + y * yAxis[1] + z * zAxis[1]);
    }

    /** A direction on the robot, as a (forward, left, up) unit vector. */
    private enum Direction {
        FRONT(1, 0, 0),
        BACK(-1, 0, 0),
        LEFT(0, 1, 0),
        RIGHT(0, -1, 0),
        UP(0, 0, 1),
        DOWN(0, 0, -1);

        private final double[] vector;

        Direction(double forward, double left, double up) {
            vector = new double[] { forward, left, up };
        }
    }
}
