// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Kinematics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static frc.robot.Kinematics.KinematicsConstants.*;

/** Add your docs here. */
public class KinematicsUtilities {

    public static double getAngleFromVector(double x, double y) {
        return Math.atan2(y, x);
    }

    public static double getNorm(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

    public static Translation2d limitVector(Translation2d vector, Translation2d limit) {
        return limitVector(vector, limit.getNorm());
    }

    public static Translation2d limitVector(Translation2d vector, double limit) {
        double vectorNorm = vector.getNorm();
        if (vectorNorm > limit) {
            return (vector.div(vectorNorm)).times(limit);
        }
        return vector;
    }

    public static boolean isInRange(double value, double limit) {
        return Math.abs(value) <= limit;
    }

    public static boolean isInRange(ChassisSpeeds speeds, double limit) {
        return Math.abs(speeds.vxMetersPerSecond) <= limit && Math.abs(speeds.vyMetersPerSecond) <= limit
                && Math.abs(speeds.omegaRadiansPerSecond) <= limit;
    }

    public static ChassisAccel getAccelFromDelta(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
        return new ChassisAccel((wantedSpeeds.vxMetersPerSecond - currentSpeeds.vyMetersPerSecond) / CYCLE_DT,
                (wantedSpeeds.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond) / CYCLE_DT,
                (wantedSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond) / CYCLE_DT);
    }

    public static class Limits {

        private static ChassisSpeeds chassisFromRest(double currentV, double wantedV, ChassisSpeeds wantedSpeeds) {

            if (wantedV < MIN_VELOCITY) { // target is standing
                return new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond);
            } else { // target is moving
                // we are moving to the required heading and accelerating, no radial limit
                double ratio = MathUtil.clamp(wantedV, currentV, currentV + MAX_DELTA_V) / wantedV;
                return new ChassisSpeeds(wantedSpeeds.vxMetersPerSecond * ratio, wantedSpeeds.vyMetersPerSecond * ratio,
                        wantedSpeeds.omegaRadiansPerSecond);
            }
        }

        private static double optimizeAngleChange(double alpha) {
            return alpha > MIN_REVERSE_ANGLE ? alpha - Math.PI : alpha + Math.PI;

        }

        public static ChassisSpeeds limitVelocities(ChassisSpeeds wantedSpeeds, ChassisSpeeds currentSpeeds) {
            double currentVelocity = Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
            double wantedVelocity = Math.hypot(wantedSpeeds.vxMetersPerSecond, wantedSpeeds.vyMetersPerSecond);

            if (currentVelocity < MIN_VELOCITY) { // we are standing
                return chassisFromRest(currentVelocity, wantedVelocity, wantedSpeeds);
            }

            if (wantedVelocity < MIN_VELOCITY) { // target is stop
                // just deaccelrate to stop
                double ratio = Math.max(currentVelocity - MAX_DELTA_V, wantedVelocity) / currentVelocity;
                return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond * ratio,
                        currentSpeeds.vyMetersPerSecond * ratio,
                        wantedSpeeds.omegaRadiansPerSecond);
            }
            // we are moving and target is moving
            double currentVelocityHeading = Math.atan2(currentSpeeds.vyMetersPerSecond,
                    currentSpeeds.vxMetersPerSecond);
            double targetVelocityHeading = Math.atan2(wantedSpeeds.vyMetersPerSecond, wantedSpeeds.vxMetersPerSecond);
            double velocityHeadingDiff = MathUtil.angleModulus(targetVelocityHeading - currentVelocityHeading);
            double targetVelocity = wantedVelocity;

            if (Math.abs(velocityHeadingDiff) < MAX_FAST_TURN_ANGLE) { // small heading change
                // accelerate to target v
                targetVelocity = MathUtil.clamp(targetVelocity, currentVelocity - MAX_DELTA_V,
                        currentVelocity + MAX_DELTA_V);
            } else if (Math.abs(velocityHeadingDiff) > MIN_REVERSE_ANGLE) { // optimization - deaccdelerate and turn the
                                                                            // other way

                targetVelocity = currentVelocity - MAX_DELTA_V;
                velocityHeadingDiff = optimizeAngleChange(velocityHeadingDiff);

            } else {
                targetVelocity = MathUtil.clamp(Math.min(MAX_ROTATION_VELOCITY, targetVelocity),
                        currentVelocity - MAX_DELTA_V, currentVelocity + MAX_DELTA_V);
            }

            if (targetVelocity < MIN_VELOCITY) {
                return new ChassisSpeeds(0, 0, wantedSpeeds.omegaRadiansPerSecond);
            }
            // calculate the maximum heading change using the target velocity and allowed
            // radial acceleration
            double maxAngleChange = 2 * (MAX_RADIAL_ACCEL / targetVelocity) * CYCLE_DT;
            // set the target angle
            velocityHeadingDiff = MathUtil.clamp(velocityHeadingDiff, -maxAngleChange, maxAngleChange);
            targetVelocityHeading = currentVelocityHeading + velocityHeadingDiff;

            // return the speeds - using target velocity and target angle
            return new ChassisSpeeds(targetVelocity * Math.cos(targetVelocityHeading),
                    targetVelocity * Math.sin(targetVelocityHeading),
                    wantedSpeeds.omegaRadiansPerSecond);
        }

    }
}