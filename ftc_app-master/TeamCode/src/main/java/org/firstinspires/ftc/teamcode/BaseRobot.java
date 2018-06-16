package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.actions.ActionList;
import org.firstinspires.ftc.teamcode.actions.Actions;
import org.firstinspires.ftc.teamcode.actions.DriveAction;
import org.firstinspires.ftc.teamcode.drive.DriveMotors;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumDriveMotors;
import org.firstinspires.ftc.teamcode.drive.mecanum.MecanumMotorPowers;
import org.firstinspires.ftc.teamcode.drive.tank.TankDriveMotorPowers;
import org.firstinspires.ftc.teamcode.drive.tank.TankDriveMotors;

/**
 * This {@code abstract class} is meant to make more succinct the corresponding {@code subclasses}.
 * <p>
 * These {@code subclasses} include a {@link TeleOp} class and an {@link Autonomous} class, i.e.
 * the classes are annotated by one of these annotations.  A few methods have already been prepared
 * so that the coder need not worry about elementary components of the code such as the drive
 * system, for which the methods {@link #tankDrive()} and {@link #mecanumDrive()} have been made.
 *
 * @see TeleOp
 * @see Autonomous
 * @see OpMode
 * @see DcMotor
 * @see Servo
 */
public abstract class BaseRobot extends OpMode {
    /**
     * This {@code double} scales the distance travelled in {@link Autonomous} to make
     * {@link DriveAction#distanceToTravelInches} actually measure in inches as desired.
     * <p>
     * NOTE: The corresponding value to this {@code double} ought to be tested so
     * that the most accurate measurements are made during {@link Autonomous}.
     *
     * @see Autonomous
     * @see DriveAction
     * @see DcMotor
     */
    private static final double DISTANCE_FACTOR = 1;

    /**
     * This {@code DriveMotors Object} (should) subsumes all the driving {@code DcMotors} declared
     * in this class.
     * <p>
     * Since {@code DriveMotors} is an {@code abstract class}, it ought to be a subclass
     * once this {@code Object} is actually created.  There are two known subclasses:
     * {@link TankDriveMotors} and {@link MecanumDriveMotors}, which contain 2 and 4
     * {@code DcMotors}, respectively.
     *
     * @see DcMotor
     */
    private DriveMotors driveMotors;

    /**
     * This {@code DcMotor} represents the mecanum wheel at the back-left corner of the robot.
     */
    private DcMotor backLeftMotor;

    /**
     * This {@code DcMotor} represents the mecanum wheel at the front-left corner of the robot.
     */
    private DcMotor frontLeftMotor;

    /**
     * This {@code DcMotor} represents the mecanum wheel at the back-right corner of the robot.
     */
    private DcMotor backRightMotor;

    /**
     * This {@code DcMotor} represents the mecanum wheel at the front-right corner of the robot.
     */
    private DcMotor frontRightMotor;

    /**
     * This {@code Servo} represents a physical arm on the robot, able to move a total of 180
     * degrees from its initial configuration (adding movements to the left and right)
     */
    private Servo arm;

    /**
     * This {@code Servo} represents a physical arm on the left side of the robot, able to move
     * a total of 180 degrees from its initial configuration (adding movements to the left and
     * right).
     * <p>
     * The word <i>synchronized</i> suggests that this arm ought to move simultaneously
     * with another arm, i.e., {@link #rightSynchronizedArm}.
     */
    private Servo leftSynchronizedArm;

    /**
     * This {@code Servo} represents a physical arm on the right side of the robot, able to move
     * a total of 180 degrees from its initial configuration (adding movements to the left and
     * right).
     * <p>
     * The word <i>synchronized</i> suggests that this arm ought to move simultaneously
     * with another arm, i.e., {@link #leftSynchronizedArm}.
     */
    private Servo rightSynchronizedArm;

    /**
     * This {@code Method} gives the factor whereby to multiply a length in {@link Autonomous}.
     * <p>
     * In particular, telling a {@code DcMotor} to move forward 5 units will not suffice, for
     * the units attached are currently arbitrary.  Therefore, this constant acts as the
     * converter from the arbitrary unit to a well-known unit, e.g., inches and centimeters.
     *
     * @return The factor by which to multiply a distance with arbitrary units attached for {@link
     * Autonomous} movement of {@code DcMotors}.
     */
    public static double getDistanceFactor() {
        return DISTANCE_FACTOR;
    }

    /**
     * This {@code Method} generates the {@code Objects} that make the robot come to life.
     * <p>
     * {@code DcMotors}, {@code Servos}, etc. are initialized here along with any directional
     * inversions for the {@code DcMotors}.
     *
     * @see Autonomous
     * @see TeleOp
     * @see DcMotor
     * @see Servo
     * @see #hardwareMap
     */
    @Override
    public void init() {
        setBackLeftMotor(hardwareMap.get(DcMotor.class, "backLeftMotor"));
        setFrontLeftMotor(hardwareMap.get(DcMotor.class, "frontLeftMotor"));
        setBackRightMotor(hardwareMap.get(DcMotor.class, "backRightMotor"));
        setFrontRightMotor(hardwareMap.get(DcMotor.class, "frontRightMotor"));

        setArm(hardwareMap.get(Servo.class, "arm"));
        setLeftSynchronizedArm(hardwareMap.get(Servo.class, "leftSynchronizedArm"));
        setRightSynchronizedArm(hardwareMap.get(Servo.class, "rightSynchronizedArm"));

        setDriveMotors(new MecanumDriveMotors(getBackLeftMotor(), getFrontLeftMotor(),
                getBackRightMotor(), getFrontRightMotor()));

        getBackRightMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        getFrontRightMotor().setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        
    }

    /**
     * This {@code Method} assumes the declaration of two {@code DcMotors} to be used for driving.
     * <p>
     * The restriction of two {@code DcMotors} vindicates the name <i>tankDrive</i>, for that
     * drive system in particular requires precisely two {@code DcMotors}.
     * <p>
     * The way to drive the robot using this {@code Method} is simple: using {@link #gamepad1},
     * the left and right joysticks are to be used, respectively representing the left and right
     * sides of the robot.
     *
     * @see TeleOp
     * @see TankDriveMotors
     * @see TankDriveMotorPowers
     * @see DcMotor
     * @see #gamepad1
     */
    void tankDrive() {
        if(getDriveMotors() instanceof TankDriveMotors) {
            TankDriveMotors tankDriveMotors = (TankDriveMotors)(getDriveMotors());

            if(tankDriveMotors.allMotorsPresent()) {
                tankDriveMotors.getLeftMotor().setPower(gamepad1.left_stick_y);
                tankDriveMotors.getRightMotor().setPower(gamepad1.right_stick_y);
            }
        }
    }

    /**
     * This {@code Method} assumes the declaration of four {@code DcMotors} to be used for driving.
     * <p>
     * The restriction of four {@code DcMotors} vindicates the name <i>mecanumDrive</i>, for that
     * drive system in particular requires precisely four {@code DcMotors}.
     * <p>
     * The way to drive the robot using this {@code Method} is simple: using {@link #gamepad1},
     * the left and right joysticks are to be used, respectively representing the direction along
     * which to travel and rotation of the robot.
     *
     * @see TeleOp
     * @see MecanumDriveMotors
     * @see MecanumMotorPowers
     * @see DcMotor
     * @see #gamepad1
     */
    void mecanumDrive() {
        if(getDriveMotors() instanceof MecanumDriveMotors) {
            MecanumDriveMotors mecanumDriveMotors = (MecanumDriveMotors)(getDriveMotors());

            if(mecanumDriveMotors.allMotorsPresent()) {
                double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
                double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) -
                        Math.PI / 4;
                double rightX = gamepad1.right_stick_x;
                final double v1 = r * Math.cos(robotAngle) + rightX;
                final double v2 = r * Math.sin(robotAngle) - rightX;
                final double v3 = r * Math.sin(robotAngle) + rightX;
                final double v4 = r * Math.cos(robotAngle) - rightX;

                mecanumDriveMotors.getFrontLeftMotor().setPower(v2);
                mecanumDriveMotors.getFrontRightMotor().setPower(v1);
                mecanumDriveMotors.getBackLeftMotor().setPower(v4);
                mecanumDriveMotors.getBackRightMotor().setPower(v3);
            }
        }
    }

    /**
     * This {@code Method} provides the outline whereby to run the robot during the
     * {@link Autonomous} period.
     * <p>
     * This assumes that the {@code init Method} has {@code Actions} added into
     * {@code ActionList}.
     * <p>
     * This {@code Method} ought to be called in the {@code loop Method} to word properly.
     *
     * @see Autonomous
     * @see Actions
     * @see ActionList
     * @see #init()
     * @see #loop()
     */
    void runAutonomous() {
        loop();
        ActionList.execute();
    }

    /**
     * This {@code Method} gives the {@code DcMotor} that corresponds to the back-left part of the
     * robot for driving.
     *
     * @return Back-left (driving) {@code DcMotor}
     */
    public DcMotor getBackLeftMotor() {
        return backLeftMotor;
    }

    /**
     * This {@code Method} sets the representative of the back-left {@code DcMotor} used for driving
     * to attain functionality, that is, movement of that {@code DcMotor}.
     *
     * @param backLeftMotor Representative of the back-left {@code DcMotor} for driving
     */
    public void setBackLeftMotor(DcMotor backLeftMotor) {
        this.backLeftMotor = backLeftMotor;
    }

    /**
     * This {@code Method} gives the {@code DcMotor} that corresponds to the front-left part of the
     * robot for driving.
     *
     * @return Front-left (driving) {@code DcMotor}
     */
    public DcMotor getFrontLeftMotor() {
        return frontLeftMotor;
    }

    /**
     * This {@code Method} sets the representative of the front-left {@code DcMotor} used for driving
     * to attain functionality, that is, movement of that {@code DcMotor}.
     *
     * @param frontLeftMotor Representative of the front-left {@code DcMotor} for driving
     */
    public void setFrontLeftMotor(DcMotor frontLeftMotor) {
        this.frontLeftMotor = frontLeftMotor;
    }

    /**
     * This {@code Method} gives the {@code DcMotor} that corresponds to the back-right part of the
     * robot for driving.
     *
     * @return Back-right (driving) {@code DcMotor}
     */
    public DcMotor getBackRightMotor() {
        return backRightMotor;
    }

    /**
     * This {@code Method} sets the representative of the back-right {@code DcMotor} used for driving
     * to attain functionality, that is, movement of that {@code DcMotor}.
     *
     * @param backRightMotor Representative of the back-right {@code DcMotor} for driving
     */
    public void setBackRightMotor(DcMotor backRightMotor) {
        this.backRightMotor = backRightMotor;
    }

    /**
     * This {@code Method} gives the {@code DcMotor} that corresponds to the front-right part of the
     * robot for driving.
     *
     * @return Front-right (driving) {@code DcMotor}
     */
    public DcMotor getFrontRightMotor() {
        return frontRightMotor;
    }

    /**
     * This {@code Method} sets the representative of the front-right {@code DcMotor} used for driving
     * to attain functionality, that is, movement of that {@code DcMotor}.
     *
     * @param frontRightMotor Representative of the front-right {@code DcMotor} for driving
     */
    public void setFrontRightMotor(DcMotor frontRightMotor) {
        this.frontRightMotor = frontRightMotor;
    }

    /**
     * This {@code Method} gives access to the set of {@code DcMotors} used for driving, disregarding
     * the drive system, for example, tank drive and mecanum, which require 2 or 4 {@code DcMotors}
     * respectively.
     *
     * @return Set of {@code DcMotors} used for driving
     */
    public DriveMotors getDriveMotors() {
        return driveMotors;
    }

    /**
     * This {@code Method} stores the {@code DriveMotors class} which represents the {@code DcMotors}
     * that translates and rotates the entirety (hopefully) of the robot.
     *
     * @param driveMotors Representation of all the driving {@code DcMotors}
     */
    public void setDriveMotors(DriveMotors driveMotors) {
        this.driveMotors = driveMotors;
    }

    /**
     * This {@code Method} gives representative of the arbitrary {@code Servo} that is to act as an
     * arm.
     *
     * @return {@code Servo} representation of an arbitrary arm on the robot
     */
    public Servo getArm() {
        return arm;
    }

    /**
     * This {@code Method} stores a {@code Servo} to represent an arbitrary arm on the robot.
     *
     * @param arm {@Servo} representation of an arbitrary arm on the robot
     */
    public void setArm(Servo arm) {
        this.arm = arm;
    }

    public Servo getLeftSynchronizedArm() {
        return leftSynchronizedArm;
    }

    public void setLeftSynchronizedArm(Servo leftSynchronizedArm) {
        this.leftSynchronizedArm = leftSynchronizedArm;
    }

    public Servo getRightSynchronizedArm() {
        return rightSynchronizedArm;
    }

    public void setRightSynchronizedArm(Servo rightSynchronizedArm) {
        this.rightSynchronizedArm = rightSynchronizedArm;
    }
}
