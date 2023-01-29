package org.firstinspires.ftc.teamcode.drive.opmode.teleopm4;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

@Config
@TeleOp(name="Teleop Auton Test 1", group="Teleop Auton")
public class TeleOpAuton extends LinearOpMode {
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    // Target A
    Vector2d highPole1 = new Vector2d(11, -24);
    double highPole1Heading = Math.toRadians(225);

    // Target B
    Vector2d highPole2 = new Vector2d(20, 0);
    double highPole2Heading = Math.toRadians(135);

    // Target C
    Vector2d highPole3 = new Vector2d(11, 24);
    double highPole3Heading = Math.toRadians(225);

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update the drive class
            drive.update();

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("mode", currentMode);
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            switch (currentMode) {
                case DRIVER_CONTROL:
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -gamepad1.left_stick_y,
//                                    -gamepad1.left_stick_x,
//                                    -gamepad1.right_stick_x
//                            )
//                    );
                    double axial = -gamepad1.left_stick_y;  // forward, back
                    double lateral = gamepad1.right_stick_x; // turning
                    double yaw = gamepad1.left_stick_x; // side to side

                    boolean slowModeOn = false;

                    if(gamepad1.left_bumper) slowModeOn = true;
                    if(gamepad1.right_bumper) slowModeOn = false;

                    double powerModifier = slowModeOn ? 0.3 : 0.7;
                    double leftFrontPower  = (axial + lateral + yaw) * powerModifier;
                    double rightFrontPower = (axial - lateral - yaw) * powerModifier;
                    double leftBackPower   = (axial - lateral + yaw) * powerModifier;
                    double rightBackPower  = (axial + lateral - yaw) * powerModifier;

                    leftFrontDrive.setPower(leftFrontPower);
                    rightFrontDrive.setPower(rightFrontPower);
                    leftBackDrive.setPower(leftBackPower);
                    rightBackDrive.setPower(rightBackPower);

                    if (gamepad1.a) {
                        // If the A button is pressed on gamepad1, we generate a splineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(highPole1, highPole1Heading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b) {
                        // If the B button is pressed on gamepad1, we generate a lineTo()
                        // trajectory on the fly and follow it
                        // We switch the state to AUTOMATIC_CONTROL

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(highPole2, highPole2Heading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        // If Y is pressed, we turn the bot to the specified angle to reach
                        // targetAngle (by default, 45 degrees)

//                        drive.turnAsync(Angle.normDelta(targetAngle - poseEstimate.getHeading()));
//
//                        currentMode = Mode.AUTOMATIC_CONTROL;

                        Trajectory traj1 = drive.trajectoryBuilder(poseEstimate)
                                .splineTo(highPole3, highPole3Heading)
                                .build();

                        drive.followTrajectoryAsync(traj1);

                        currentMode = Mode.AUTOMATIC_CONTROL;
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    // If x is pressed, we break out of the automatic following
                    if (gamepad1.x) {
                        drive.breakFollowing();
                        currentMode = Mode.DRIVER_CONTROL;
                    }

                    // If drive finishes its task, cede control to the driver
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
        }
    }
}
