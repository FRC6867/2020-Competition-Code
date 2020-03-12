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
        public static final int FRONT_LEFT_MOTOR_CAN = 13;
        public static final int BACK_LEFT_MOTOR_CAN = 12;
        public static final int FRONT_RIGHT_MOTOR_CAN = 10;
        public static final int BACK_RIGHT_MOTOR_CAN = 11;

        public static final boolean LEFT_MOTOR_INVERTED = false; // Undefined
        public static final boolean RIGHT_MOTOR_INVERTED = true; // Undefined

        // Encoders
        public static final int[] LEFT_DRIVE_ENCODER_PINS = {0, 1};
        public static final int[] RIGHT_DRIVE_ENCODER_PINS = {2, 3};

        public static final double LEFT_ENCODER_TICK_DISTANCE = 0.0103545; // Undefined
        public static final double RIGHT_ENCODER_TICK_DISTANCE = 0.0103237; // Undefined

        public static final boolean LEFT_ENCODER_INVERTED = true;
        public static final boolean RIGHT_ENCODER_INVERTED = false;

        // Ultrasonic sensor pins
        public static final int ULTRASONIC_TRIGGER_PIN = 8; // Undefined
        public static final int ULTRASONIC_ECHO_PIN = 9; // Undefined

        // Speed constants
        public static final double DEFAULT_SPEED_THROTTLE = 0.7; // Default speed multiplier
        public static final double MIN_SPEED_THRESHOLD = 0.1; // Inputs under will be ignored
        public static final double FINE_CONTROL_SENSITIVITY = 2.5;
        public static final double ARTIFICIAL_COAST_NUM = 1.05;
        public static final double JOYSTICK_SENSITIVITY_MOD = 1/2;
        public static final boolean COAST_DEFAULT_MODE = false;


        // PIDs
        public static final double TURN_CORRECTION_P = 0.25; // Undefined

        // For turning a specific amount of degrees
        public static final double TURN_P = 0.03;  // Undefined
        public static final double TURN_I = 0.001; // Undefined
        public static final double TURN_D = 0.0035; // Undefined

        // For moving for a specific distance
        public static final double DRIVE_P = 0.01; // Undefined
        public static final double DRIVE_I = 0.0; // Undefined
        public static final double DRIVE_D = 0.0; // Undefined

        public static final double TURN_TARGET_TOLERANCE = 0.5;
        public static final double DRIVE_TARGET_TOLERANCE = 1;
    }

    public static final class IntakeConstants {
        // User input
        public static final int COLLECTOR_BUTTON_ID = 6; // Right bumper
        public static final int ARM_MANUAL_UP_BUTTON_ID = 3; // Right trigger
        public static final int ARM_MANUAL_DOWN_BUTTON_ID = 2; // Left trigger

        // Motors
        public static final int COLLECTOR_MOTOR_CAN = 20;
        public static final int ARM_MOTOR_CAN = 21;

        public static final boolean COLLECTOR_MOTOR_INVERTED = true;
        public static final boolean ARM_MOTOR_INVERTED = true;

        public static final int[] ARM_ENCODER_PINS = {4, 5};
        public static final boolean ARM_ENCODER_INVERTED  = true;

        // PID
        public static final double ARM_P = 0.0; // Undefined
        public static final double ARM_I = 0.0; // Undefined
        public static final double ARM_D = 0.0; // Undefined
        public static final double ARM_F = 0.0; // Undefined

        public static final int ARM_0_DEGREE_POS = 100; // Undefined
        public static final int ARM_FINAL_POS = 2000; // Undefined
        public static final int ARM_FINAL_POS_DEGREES = 90;
        public static final int ARM_DEGREE_UP_POS = 20; // Supposed to be 0, when tuned
        public static final int ARM_DEGREE_DOWN_POS = 70; // Supposed to be 90, when tuned
        public static final int ARM_TARGET_TOLERANCE = 3; // Allowed innacuracy

        // Speeds
        public static final double COLLECTOR_SPEED = 0.95; // Or 0.75
        public static final double ARM_MANUAL_SPEED = 0.7;
    }

    public static final class IndexerConstants {
        // User input
        public static final int RIGHT_FORWARD_BUTTON_ID = 6; // Right bumper
        public static final int RIGHT_BACKWARD_BUTTON_ID = 3; // Right trigger
        public static final int LEFT_FORWARD_BUTTON_ID = 5; // Left bumper
        public static final int LEFT_BACKWARD_BUTTON_ID = 2; // Left trigger

        // Motors
        public static final int RIGHT_MOTOR_CAN = 30;
        public static final int LEFT_MOTOR_CAN = 31;

        public static final boolean RIGHT_MOTOR_INVERTED = false;
        public static final boolean LEFT_MOTOR_INVERTED = true;

        // Speeds

        public static final double INDEXER_SPEED = 0.7;

        // Ball detection
        public static final int BALL_CLOSE_DISTANCE = 100; // Undefined
    }

    public static final class ShooterConstants {
        // User input
        public static final int SHOOTER_TOGGLE_BUTTON_ID = 1; // 'A' button
        public static final int SHOOTER_SPEED_BUTTON_ID = 3; // 'X' button
        public static final int FEEDER_BUTTON_ID = 2; // 'B' button
            
        public static final int VOMIT_BUTTON_1_ID = 9; // Left joystick button
        public static final int VOMIT_BUTTON_2_ID = 10; // Right joystick button

        // Motor CANs
        public static final int FEEDER_MOTOR_CAN = 42;
        public static final int SHOOTER_MOTOR_1_CAN = 40;
        public static final int SHOOTER_MOTOR_2_CAN = 41;

        public static final boolean SHOOTER_MOTORS_INVERTED = true;
        public static final boolean FEEDER_MOTOR_INVERTED = true;

        // Encoder
        public static final int[] SHOOTER_ENCODER_PINS = {6, 7};
        public static final boolean SHOOTER_ENCODER_INVERTED = true;
        public static final int SHOOTER_ENCODER_CPR = 2048;


        // Shooter PID
        public static double SHOOTER_P = 0.0005; // Undefined
        public static double SHOOTER_I = 0.000; // Undefined
        public static double SHOOTER_D = 0.0; // Undefined
        public static double SHOOTER_F = 0.000222222222; // Undefined

        // Speeds
        public static double SHOOTER_TARGET_RPM = 3600; // Adjustable during run
        public static double SHOOTER_VOMIT_TARGET_RPM = 1500; // Undefined
        public static double SHOOTER_TARGET_RPM_TOLERANCE = 75;

        public static double FEEDER_SPEED = 0.8;
    }

    public static final class ClimbConstants {
        // User input
        public static final int RELEASE_CLIMB_BUTTON_ID = 7; // Back button
        public static final int CLIMB_UP_BUTTON_ID = 8; // Start button

        // Motor CANs
        public static final int CLIMB_MOTOR_1_CAN = 50;
        public static final int CLIMB_MOTOR_2_CAN = 51; // TODO: Can ID this Talon

        public static final boolean CLIMB_MOTOR_1_INVERTED = false; // Undefined
        public static final boolean CLIMB_MOTOR_2_INVERTED = false; // Undefined

        public static final double CLIMB_MOTOR_1_SPEED = 0.3; // Undefined
        public static final double CLIMB_MOTOR_2_SPEED = 0.3; // Undefined
    }

    public static final class VisionConstants {
        // Ports
        public static final int MAIN_CAMERA_PORT_NUM = 0;
        public static final int CLIMB_CAMERA_PORT_NUM = 1;

        // Names
        public static final String MAIN_CAMERA_NAME = "Main Camera";
        public static final String CLIMB_CAMERA_NAME = "Climb Camera";

        // Main cam stats
        public static final int MAIN_CAMERA_WIDTH = 100;
        public static final int MAIN_CAMERA_HEIGTH = 80;
        public static final int MAIN_CAMERA_FPS = 20;

        // Timing
        public static final int CAMERA_TURN_ON_TIME = 30; // Seconds before the end of the match
    }

    public static final class DecorConstants {
        public static final int LED_PIN_ID = 0; // Undefined

        public static final double DEFAULT_LED_STATE = 0;
    }

    public static final class AutoConstants {
        public static final class Auto1Constants {
            public static final double DRIVE_TIME = 3.5;
            public static final double DRIVE_SPEED = 0.4;
            public static final double SHOOT_DELAY = 0.3;
            public static final double SHOOT_TIME = 10;
        }

        public static final class Auto2Constants {
            public static final double DRIVE_1_DISTANCE = 20;
            public static final double TURN_DEGREES = 90;
            public static final double DRIVE_2_DISTANCE = 50;
            public static final double WALL_DRIVE_TIME = 2;
        }
    }
}
