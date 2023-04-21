/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.commands.conveyor.modes;

import com.stuypulse.robot.subsystems.Conveyor;
import com.stuypulse.robot.subsystems.Conveyor.Direction;

import java.util.function.Consumer;

public enum ConveyorMode {
    INDEX(
            (Conveyor conveyor) -> {
                /*** Gandalf logic ***/

                // Eject if you have wrong ball
                if (conveyor.hasOpponentBall()) {
                    conveyor.setGandalfDirection(Direction.REVERSE);
                }

                // Stop if you already have ball
                else if (conveyor.hasTopBeltBall()) {
                    conveyor.setGandalfDirection(Direction.STOPPED);
                }

                // Accept Alliance Ball if no ball on top
                else if (conveyor.hasAllianceBall()) {
                    conveyor.setGandalfDirection(Direction.FORWARD);
                }

                // If you were ejecting and there is no longer a ball, stop
                else if (conveyor.getGandalfDirection() == Direction.REVERSE) {
                    conveyor.setGandalfDirection(Direction.STOPPED);
                }

                /*** Top belt logic ***/

                // Stop if you already have ball
                if (conveyor.hasTopBeltBall()) {
                    conveyor.setTopBeltDirection(Direction.STOPPED);
                }

                // Run upwards if you have an alliance ball
                else if (conveyor.hasAllianceBall()) {
                    conveyor.setTopBeltDirection(Direction.FORWARD);
                }

                // Stop Ejecting Once Done
                else if (conveyor.getTopBeltDirection() == Direction.REVERSE) {
                    conveyor.setTopBeltDirection(Direction.STOPPED);
                }
            }),

    FORCE_INTAKE(
            (Conveyor conveyor) -> {
                conveyor.setTopBeltDirection(
                        conveyor.hasTopBeltBall() ? Direction.STOPPED : Direction.FORWARD);
                conveyor.setGandalfDirection(Direction.FORWARD);
            }),

    SHOOT(
            (Conveyor conveyor) -> {
                conveyor.setTopBeltDirection(
                        conveyor.hasTopBeltBall() ? Direction.FORWARD_SLOW : Direction.FORWARD);
                conveyor.setGandalfDirection(
                        conveyor.hasOpponentBall() ? Direction.REVERSE : Direction.FORWARD);
            }),

    SHOOT_SLOW(
            (Conveyor conveyor) -> {
                conveyor.setTopBeltDirection(Direction.FORWARD_SLOW);
                conveyor.setGandalfDirection(
                        conveyor.hasOpponentBall() ? Direction.REVERSE : Direction.FORWARD);
            }),

    SHOOT_TOP(
            (Conveyor conveyor) -> {
                conveyor.setTopBeltDirection(Direction.FORWARD);
                conveyor.setGandalfDirection(Direction.STOPPED);
            }),

    SEMI_AUTO(
            (Conveyor conveyor) -> {
                boolean shouldStop = conveyor.hasNewBall();
                conveyor.setTopBeltDirection(shouldStop ? Direction.STOPPED : Direction.FORWARD_SLOW);
                conveyor.setGandalfDirection(shouldStop ? Direction.STOPPED : Direction.FORWARD);
            }),

    EJECT(
            (Conveyor conveyor) -> {
                conveyor.setGandalfDirection(Direction.REVERSE);
                conveyor.setTopBeltDirection(Direction.REVERSE);
            }),

    STOPPED(
            (Conveyor conveyor) -> {
                conveyor.setGandalfDirection(Direction.STOPPED);
                conveyor.setTopBeltDirection(Direction.STOPPED);
            }),

    DEFAULT(INDEX.method);

    private final Consumer<Conveyor> method;

    public void run(Conveyor conveyor) {
        method.accept(conveyor);
    }

    private ConveyorMode(Consumer<Conveyor> method) {
        this.method = method;
    }
}
