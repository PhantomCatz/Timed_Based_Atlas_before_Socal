package frc.Autonomous.Actions;

public interface ActionBase {
    // is called once when the action is started
    public void init();

    // calculates if the action is finished
    public boolean isFinished();

    // is continuously called during the action
    public void update();

    // is called once when the action ends
    public void end();
}