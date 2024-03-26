package org.firstinspires.ftc.teamcode.Helpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Box;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Intake;

/**
 * Simple static field serving as a storage medium for the bot's pose.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 *
 * Store some other custom functions as well
 */

@Config
public class Helpers {

    public static Pose2d initAutoPose = new Pose2d(0, 0, Math.toRadians(-270));
    public static Pose2d defaultTelePose = new Pose2d(0, 0, 0);


    public static class RaceParallelCommand implements Action {
        private final Action[] actions;

        public RaceParallelCommand(Action... actions) {
            this.actions = actions;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean finished = true;
            for (Action action : actions) finished = finished && action.run(telemetryPacket);
            return finished;
        }

        @Override
        public void preview(@NonNull Canvas canvas) {
            for (Action action : actions) action.preview(canvas);
        }
    }

    }



