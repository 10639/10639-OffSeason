package org.firstinspires.ftc.teamcode.Subsystems.Helpers;

public class Constants {

    /** ======= CONSTANTS FOR LIFT  ======= **/

    public static final double COUNTS_PER_MOTOR_REV    = 103.8 ;   // eg: GoBILDA 1620m RPM Yellow Jacket
    public static final double DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES   = 1.673 ;     // For figuring circumference
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    public enum SlidePositions {
        DOWN(0),
        LOW((int) (17 * COUNTS_PER_INCH)),
        MEDIUM((int) (19 * COUNTS_PER_INCH)),
        HIGH((int) (20 * COUNTS_PER_INCH));
        public final int position;

        SlidePositions(int position) {this.position = position;}
        public int getTicks() {return this.position;}
    }

    public enum SlidesPIDF {
        Kp(0.1),
        Ki(0.0),
        Kd(0.00001),
        Kf(0.04);
        public final double value;
        SlidesPIDF(double value) {this.value = value;}
        public double getValue() {return this.value;}
    }

    //Proportional, Integral, Derivative gains.
    //P -> Ability to Reach Target Position
    //I -> Leave at 0
    //D -> Dampener to counteract any oscillations
    //F -> Ability to counteract gravity and maintain its position (going up, down, and in place)
    // Feedforward component (F) -> Since we're doing this for a lift; we'll do a G value (gravity).
    //Refer to https://www.ctrlaltftc.com/feedforward-control#slide-gravity-feedforward

    public enum VelocityConfig {
        LIFT_UP(1.0),
        LIFT_TERMINAL(-0.5),
        DRIVETRAIN_MIN_SPEED(0.5),
        DRIVETRAIN_MAX_VELO(0.9);
        public final double velocity;

        VelocityConfig(double velocity) {this.velocity = velocity;}
        public double getVelocity() {return this.velocity;}
    }


    /** ======= CONSTANTS FOR ARM  ======= **/

    public static final double closeArm = 1; //Claw closed position
    public static final double openArm = 0; //Claw opened position
    public static final double rotationScore = 0;
    public static final double rotationIdle = 1;
    public static final double leftArm_Idle = 0.7;
    public static final double rightArm_Idle = 0;
    public static final double leftArm_Score = 0;
    public static final double rightArm_Score = 1;



    /** ======= CONSTANTS FOR INTAKE  ======= **/
    public static final double Sweep = -1;
    public static final double reverseSweep = 1;
    public static final double intakeExtend = 1;
    public static final double intakeRetract = 0;
    public static final double boxSweep = -1;
    public static final double boxReverseSweep = 1;
    public static final double terminatePower = 0;



    /** ======= CONSTANTS FOR VISION  ======= **/
    public static final double CONFIDENCE = 0.20;

    /** ======= CONSTANTS FOR DISTANCE SENSOR (CM) ======= **/
    public static final double EMPTY_BOX_HIGH = 15;

    public static final double ONE_PIXEL_HIGH = 9;

    public static final double FULL_BOX_HIGH = 3;









}