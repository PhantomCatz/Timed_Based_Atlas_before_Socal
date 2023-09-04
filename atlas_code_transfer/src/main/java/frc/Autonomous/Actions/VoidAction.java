package frc.Autonomous.Actions;

// runs whatever is given in execute()
public class VoidAction implements ActionBase{
    public interface VoidInterface
    {
        public void execute();
    }

    private VoidInterface action;

    public VoidAction(VoidInterface action)
    {
        this.action = action;
    }

    @Override
    public void init() {
        action.execute();        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void update() {}

    @Override
    public void end() {}
}