/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class PIDConstants{
        public static final double TURN_CORRECTION_P = 0.25;

        // For turning a specific amount of degrees
        public static double TURN_P = 0.05;
        public static double TURN_I = 0.0;
        public static double TURN_D = 0.006;

        // For moving for a specific distance
        public static double DRIVE_FOR_P = 0.01;
        public static double DRIVE_FOR_I = 0.0;
        public static double DRIVE_FOR_D = 0.0;

        // For moving to a specific distance from an object
        public static double DRIVE_UNTIL_P = 0.01;
        public static double DRIVE_UNTIL_I = 0.0;
        public static double DRIVE_UNTIL_D = 0.0;

        // To center the Krab
        public static double KRAB_P = 0.01;
        public static double KRAB_I = 0.0;
        public static double KRAB_D = 0.0;
    }

    public static final class ComponentIDConstants {
        // Motor CANs
        public static final int FRONT_LEFT_DRIVE_CAN = 11;
        public static final int BACK_LEFT_DRIVE_CAN = 10;
        public static final int FRONT_RIGHT_DRIVE_CAN = 21;
        public static final int BACK_RIGHT_DRIVE_CAN = 20;
        
        public static final int KRAB_SLIDE_MOTOR_CAN = 40;

        public static final int CLIMB_MOTOR_CAN = 01; // Motor is disabled

        // Encoder pins
        public static final int LEFT_DRIVE_ENCODER_PIN_1 = 1;
        public static final int LEFT_DRIVE_ENCODER_PIN_2 = 2;
        public static final int RIGHT_DRIVE_ENCODER_PIN_1 = 7;
        public static final int RIGHT_DRIVE_ENCODER_PIN_2 = 8;

        // Ultrasonic sensor pins
        public static final int ULTRASONIC_TRIGGER_PIN = 4;
        public static final int ULTRASONIC_ECHO_PIN = 5;
    }

    public static final class ControllerConstants {
        // Triggers
        public static final int KRAB_RIGHT_BUTTON = 3; // Right Trigger
        public static final int KRAB_LEFT_BUTTON = 2; // Left Trigger
        public static final int CENTER_KRAB_BUTTON = 0; // Currently undefined

        // Left stick axis
        public static final int LEFT_STICK_Y_AXIS = 1;
        public static final int LEFT_STICK_X_AXIS = 0;

        // Right stick axis
        public static final int RIGHT_STICK_Y_AXIS = 5;
        public static final int RIGHT_STICK_X_AXIS = 4;

        // Climb buttons
        public static final int CLIMB_UP_BUTTON = 4; // Y button
        public static final int CLIMB_DOWN_BUTTON = 1; // A button
    }

    public static final class SpeedConstants {
        public static final double MASTER_THROTTLE = 0.7;

        public static final double KRAB_SPEED = 0.35;
        public static final int KRAB_RIGHT_STOP = -5000; // Krab limits
        public static final int KRAB_LEFT_STOP = 5000;
    }
}
