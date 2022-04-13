package org.firstinspires.ftc.teamcode.Auto.Good;

import org.firstinspires.ftc.teamcode.Auto.AutoBase;

public class StateMachineTest extends AutoBase {

    public enum Auto{
        a1{
            public Auto nextState()
            {
                return a2;
            }
        },
        a2{
            public Auto nextState()
            {
                return a3;
            }
        },
        a3{
            public Auto nextState()
            {
                return a4;
            }
        },
        a4{
            public Auto nextState()
            {
                return this;
            }
        };

        public abstract Auto nextState();
    }
    public void runOpMode()
    {
        startUp();
        waitForStart();
    }
}
