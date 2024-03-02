package org.firstinspires.ftc.teamcode.Subsystems.Trajectories;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Builder {

    public Pose2d initPose;

    public Pose2d centerRetractPos;
    public Pose2d leftRetractPos;
    public Pose2d rightRetractPos;

    public Vector2d midwayVector;
    public Vector2d centerVector;
    public Vector2d leftVector;
    public Vector2d rightVector;

    public Vector2d centerScoringVector;
    public Vector2d leftScoringVector;
    public Vector2d rightScoringVector;


    public Vector2d parkingPose;
    public double backBoard_X = 48;

    public Action trajCenter;
    public Action trajLeft;
    public Action trajRight;

    public void calculatePoses(String alliance, String side, Telemetry telemetry) {

        switch(alliance) {
            case "BLUE":
                if(side == "LEFT") {
                    initPose = new Pose2d(13, 58, Math.toRadians(-270));
                    midwayVector = new Vector2d(initPose.position.x, 35);

                    leftVector = new Vector2d(22, midwayVector.y);
                    leftRetractPos = new Pose2d(leftVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    leftScoringVector = new Vector2d(backBoard_X, (leftRetractPos.position.y) - 1);

                    centerVector = new Vector2d(initPose.position.x, 30);
                    centerRetractPos = new Pose2d(midwayVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    centerScoringVector = new Vector2d(backBoard_X, (centerRetractPos.position.y) - 7);

                    rightVector = new Vector2d(2, midwayVector.y);
                    rightRetractPos = new Pose2d(rightVector.x + 10, midwayVector.y + 2, initPose.heading.toDouble());
                    rightScoringVector = new Vector2d(backBoard_X, (rightRetractPos.position.y) - 11);

                    parkingPose = new Vector2d(backBoard_X,58);
                } else {
                    initPose = new Pose2d(13, 58, Math.toRadians(-270));
                    midwayVector = new Vector2d(initPose.position.x, 35);

                    leftVector = new Vector2d(22, midwayVector.y);
                    leftRetractPos = new Pose2d(leftVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    leftScoringVector = new Vector2d(backBoard_X, (leftRetractPos.position.y) - 1);

                    centerVector = new Vector2d(initPose.position.x, 30);
                    centerRetractPos = new Pose2d(midwayVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    centerScoringVector = new Vector2d(backBoard_X, (centerRetractPos.position.y) - 7);

                    rightVector = new Vector2d(2, midwayVector.y);
                    rightRetractPos = new Pose2d(rightVector.x + 10, midwayVector.y + 2, initPose.heading.toDouble());
                    rightScoringVector = new Vector2d(backBoard_X, (rightRetractPos.position.y) - 11);

                    parkingPose = new Vector2d(backBoard_X,58);
                }
                break;
            case "RED":
                if(side == "left") {
                    initPose = new Pose2d(13, 58, Math.toRadians(-270));
                    midwayVector = new Vector2d(initPose.position.x, 35);

                    leftVector = new Vector2d(22, midwayVector.y);
                    leftRetractPos = new Pose2d(leftVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    leftScoringVector = new Vector2d(backBoard_X, (leftRetractPos.position.y) - 1);

                    centerVector = new Vector2d(initPose.position.x, 30);
                    centerRetractPos = new Pose2d(midwayVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    centerScoringVector = new Vector2d(backBoard_X, (centerRetractPos.position.y) - 7);

                    rightVector = new Vector2d(2, midwayVector.y);
                    rightRetractPos = new Pose2d(rightVector.x + 10, midwayVector.y + 2, initPose.heading.toDouble());
                    rightScoringVector = new Vector2d(backBoard_X, (rightRetractPos.position.y) - 11);

                    parkingPose = new Vector2d(backBoard_X,58);
                } else {
                    initPose = new Pose2d(13, 58, Math.toRadians(-270));
                    midwayVector = new Vector2d(initPose.position.x, 35);

                    leftVector = new Vector2d(22, midwayVector.y);
                    leftRetractPos = new Pose2d(leftVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    leftScoringVector = new Vector2d(backBoard_X, (leftRetractPos.position.y) - 1);

                    centerVector = new Vector2d(initPose.position.x, 30);
                    centerRetractPos = new Pose2d(midwayVector.x, midwayVector.y + 7, initPose.heading.toDouble());
                    centerScoringVector = new Vector2d(backBoard_X, (centerRetractPos.position.y) - 7);

                    rightVector = new Vector2d(2, midwayVector.y);
                    rightRetractPos = new Pose2d(rightVector.x + 10, midwayVector.y + 2, initPose.heading.toDouble());
                    rightScoringVector = new Vector2d(backBoard_X, (rightRetractPos.position.y) - 11);

                    parkingPose = new Vector2d(backBoard_X,58);
                }
                break;
        }
        telemetry.addLine("--- Trajectory Builder Detection ---");
        telemetry.addData("Vectors & Poses Calculated", "Side: " + alliance + " " + side);

    }

}
