package org.firstinspires.ftc.teamcode.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Action {
    private ElapsedTime runtime = new ElapsedTime();

    private int durationMilliseconds;

    private boolean hasStarted;
    private boolean isFinished;

    public Action() {
        this(0);
    }

    public Action(int durationMilliseconds) {
        setDurationMilliseconds(durationMilliseconds);
    }

    public void start() {
        if(!hasStarted()) {
            setHasStarted(true);
            getRuntime().reset();
        }
    }

    public void execute() {
        setFinished(getRuntime().milliseconds() >= getDurationMilliseconds());
    }

    public ElapsedTime getRuntime() {
        return runtime;
    }

    public void setRuntime(ElapsedTime runtime) {
        this.runtime = runtime;
    }

    public int getDurationMilliseconds() {
        return durationMilliseconds;
    }

    public void setDurationMilliseconds(int durationMilliseconds) {
        this.durationMilliseconds = durationMilliseconds;
    }

    public boolean hasStarted() {
        return hasStarted;
    }

    public void setHasStarted(boolean hasStarted) {
        this.hasStarted = hasStarted;
    }

    public boolean isFinished() {
        return isFinished;
    }

    public void setFinished(boolean finished) {
        isFinished = finished;
    }
}
