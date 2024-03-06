package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 14)
                .build();

         Pose2d initPose;

         Pose2d centerRetractPos;
         Pose2d leftRetractPos;
         Pose2d rightRetractPos;

         Vector2d midwayVector;
         Vector2d centerVector;
         Vector2d leftVector;
         Vector2d rightVector;

         Vector2d centerScoringVector;
         Vector2d leftScoringVector;
         Vector2d rightScoringVector;


         Vector2d parkingPose;
         double backBoard_X = 48;

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
        Pose2d test = new Pose2d(36 ,58, initPose.heading.toDouble());


        myBot.runAction(myBot.getDrive().actionBuilder(initPose)
                .setReversed(true)
                .splineTo(centerVector, Math.toRadians(-90))
                .splineToLinearHeading(centerRetractPos, Math.toRadians(-90))
                .strafeTo(new Vector2d( (centerRetractPos.position.x) + 4 , centerRetractPos.position.y))
                .setReversed(true)
                .splineTo(centerScoringVector, Math.toRadians(0))
                .waitSeconds(1)
                .waitSeconds(1)
                .waitSeconds(1)
                .waitSeconds(1)
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(backBoard_X - 2,60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(backBoard_X,60), Math.toRadians(0))
                .waitSeconds(0.5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1f)
                .addEntity(myBot)
                .start();
    }
}