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
    public static final class DriveTrainConstants {
        // User input
        public static final int LEFT_STICK_Y_AXIS_ID = 1; // Left stick Y
        public static final int RIGHT_STICK_Y_AXIS_ID = 5; // Right stick Y

        public static final int FINE_CONTROL_BUTTON_ID = 5; // Left bumper

        // Drive motor CANs
        public static final int FRONT_LEFT_MOTOR_CAN = 10;
        public static final int BACK_LEFT_MOTOR_CAN = 11;
        public static final int FRONT_RIGHT_MOTOR_CAN = 20;
        public static final int BACK_RIGHT_MOTOR_CAN = 21;

        public static final boolean LEFT_MOTOR_INVERTED = false;
        public static final boolean RIGHT_MOTOR_INVERTED = true;

        // Encoders
        public static final int[] LEFT_DRIVE_ENCODER_PINS = {1, 2};
        public static final int[] RIGHT_DRIVE_ENCODER_PINS = {7, 8};

        public static final boolean LEFT_ENCODER_INVERTED = true;
        public static final boolean RIGHT_ENCODER_INVERTED = false;

        // Ultrasonic sensor pins
        public static final int ULTRASONIC_TRIGGER_PIN = 0; // Undefined
        public static final int ULTRASONIC_ECHO_PIN = 0; // Undefined


        // PIDs
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


        // Speed constants
        public static final double SPEED_THROTTLE = 0.5; // Speed multiplier
    }

    public static final class IntakeConstants {
        // User input
        public static final int INTAKE_TOGGLE_BUTTON_ID = 6; // Right bumper

        // Motor CANs
        public static final int INTAKE_COLLECTOR_MOTOR_CAN = 30;
        public static final int INTAKE_MOVER_MOTOR_CAN = 31;
    }

    public static final class IndexerConstants {
        // User input
        public static final int INDEXER_TOGGLE_BUTTON_ID = 4; // Y button

        // Motor CANs
        public static final int INDEXER_MOTOR_CAN = 40;
    }

    public static final class ShooterConstants {
        // User input
        public static final int SHOOTER_TOGGLE_BUTTON_ID = 1; // A button

        // Motor CANs
        public static final int FEEDER_MOTOR_CAN = 52;
        public static final int SHOOTER_MOTOR_1_CAN = 50;
        public static final int SHOOTER_MOTOR_2_CAN = 51;

        // Encoder pins
        public static final int[] SHOOTER_ENCODER_PINS  = {0, 0}; // Undefined

        // Shooter PID
        public static double SHOOTER_P = 0.1;
        public static double SHOOTER_I = 0.0;
        public static double SHOOTER_D = 0.0;
    }

    public static final class ClimbConstants {
        // User input
        public static final int CLIMB_UP_BUTTON_ID = 7; // Back button
        public static final int CLIMB_DOWN_BUTTON_ID = 8; // Start button

        // Motor CANs
        public static final int CLIMB_MOTOR_CAN = 60;
    }
}
