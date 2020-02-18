/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/*
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveTrainConstants {
        // User input
        public static final int LEFT_STICK_Y_AXIS_ID = 1; // Left stick Y
        public static final int RIGHT_STICK_Y_AXIS_ID = 5; // Right stick Y

        public static final int FINE_CONTROL_BUTTON_ID = 5; // Left bumper

        // Motors
        public static final int FRONT_LEFT_MOTOR_CAN = 13; // These all need to be verified
        public static final int BACK_LEFT_MOTOR_CAN = 12; // once they are placed on the robot
        public static final int FRONT_RIGHT_MOTOR_CAN = 10;
        public static final int BACK_RIGHT_MOTOR_CAN = 11;

        public static final boolean LEFT_MOTOR_INVERTED = true; // Undefined
        public static final boolean RIGHT_MOTOR_INVERTED = false; // Undefined

        // Encoders
        public static final int[] LEFT_DRIVE_ENCODER_PINS = {1, 2}; // Undefined
        public static final int[] RIGHT_DRIVE_ENCODER_PINS = {3, 4}; // Undefined

        public static final double LEFT_ENCODER_TICK_DISTANCE = 1; // Undefined
        public static final double RIGHT_ENCODER_TICK_DISTANCE = 1; // Undefined

        public static final boolean LEFT_ENCODER_INVERTED = true; // Undefined
        public static final boolean RIGHT_ENCODER_INVERTED = false; // Undefined

        // Ultrasonic sensor pins
        public static final int ULTRASONIC_TRIGGER_PIN = 5; // Undefined
        public static final int ULTRASONIC_ECHO_PIN = 6; // Undefined

        // Speed constants
        public static final double DEFAULT_SPEED_THROTTLE = 0.3; // Default speed multiplier
        public static final double MIN_SPEED_THRESHOLD = 0.1; // Inputs under will be ignored


        // PIDs
        public static final double TURN_CORRECTION_P = 0.25; // Undefined

        // For turning a specific amount of degrees
        public static double TURN_P = 0.05;  // Undefined
        public static double TURN_I = 0.0; // Undefined
        public static double TURN_D = 0.006; // Undefined

        // For moving for a specific distance
        public static double DRIVE_P = 0.01; // Undefined
        public static double DRIVE_I = 0.0; // Undefined
        public static double DRIVE_D = 0.0; // Undefined

        public static double TURN_TARGET_TOLERANCE = 0.5;
        public static double DRIVE_TARGET_TOLERANCE = 1;
    }

    public static final class IntakeConstants {
        // User input
        public static final int INTAKE_BUTTON_ID = 6; // Right bumper

        // Motors
        public static final int INTAKE_COLLECTOR_MOTOR_CAN = 20; // Undefined
        public static final int INTAKE_MOVER_MOTOR_CAN = 21; // Undefined

        public static final boolean INTAKE_COLLECTOR_MOTOR_INVERTED = false; // Undefined
        public static final boolean INTAKE_MOVER_MOTOR_INVERTED = false; // Undefined
    }

    public static final class IndexerConstants {
        // User input
        public static final int INDEXER_BUTTON_ID = 4; // 'Y' button

        // Motors
        public static final int INDEXER_MOTOR_CAN = 30; // Undefined

        public static final boolean INDEXER_MOTOR_INVERTED = false; // Undefined
        public static final double INDEXER_MOTOR_SPEED = 0.3;

        // Ball color
        public static final double BALL_COLOR_R = 0.361; // Undefined
        public static final double BALL_COLOR_B = 0.524; // Undefined
        public static final double BALL_COLOR_G = 0.113; // Undefined

        public static final int BALL_CLOSE_DISTANCE = 100; // Undefined
    }

    public static final class ShooterConstants {
        // User input
        public static final int SHOOTER_TOGGLE_BUTTON_ID = 1; // 'A' button
        public static final int FEEDER_BUTTON_ID = 2; // 'X' or 'B' button

        // Motor CANs
        public static final int FEEDER_MOTOR_CAN = 42; // Undefined
        public static final int SHOOTER_MOTOR_1_CAN = 40;
        public static final int SHOOTER_MOTOR_2_CAN = 41;

        public static final boolean SHOOTER_MOTORS_INVERTED = false; // Undefined
        public static final boolean FEEDER_MOTOR_INVERTED = false; // Undefined

        // Encoder
        public static final int[] SHOOTER_ENCODER_PINS  = {7, 8}; // Undefined
        public static final boolean SHOOTER_ENCODER_INVERTED = false; // Undefined
        public static final int SHOOTER_ENCODER_TICKS_PER_ROTATION = 8000;

        // Shooter PID
        public static double SHOOTER_P = 0.1; // Undefined
        public static double SHOOTER_I = 0.0; // Undefined
        public static double SHOOTER_D = 0.0; // Undefined

        // Speeds
        public static double SHOOTER_TARGET_RPM = 6000; // Undefined
        public static double SHOOTER_TARGET_RPM_TOLERANCE = 500; // 500 for now

        public static double FEEDER_SPEED = 0.3; // Slow for now
    }

    public static final class ClimbConstants {
        // User input
        public static final int RELEASE_CLIMB_BUTTON_ID = 7; // Back button
        public static final int CLIMB_UP_BUTTON_ID = 8; // Start button

        // Motor CANs
        public static final int CLIMB_MOTOR_CAN = 50; // Undefined

        public static final boolean CLIMB_MOTOR_INVERTED = false; // Undefined
        public static final double CLIMB_MOTOR_SPEED = 0.5; // Slow for now
    }

    // Vomit buttons
    public static final int VOMIT_BUTTON_1_ID = 1; // Undefined
    public static final int VOMIT_BUTTON_2_ID = 1; // Undefined
}
