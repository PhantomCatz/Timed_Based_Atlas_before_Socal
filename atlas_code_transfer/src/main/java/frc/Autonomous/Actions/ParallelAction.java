package frc.Autonomous.Actions;

import java.util.ArrayList;
import java.util.List;

/**
 * Runs all give actions in parallel. All actions are started then updated until all actions
 * report being done.
 */
public class ParallelAction implements ActionBase{

    private final ArrayList<ActionBase> actions;

    public ParallelAction(List<ActionBase> actions)
    {
        this.actions = new ArrayList<>(actions);
    }

    @Override
    public void init() 
    {
        for(ActionBase action : actions){
            action.init();
        }
    }

    @Override
    public boolean isFinished() {
        for(ActionBase action : actions)
        {
            if(!action.isFinished())
            {
                return false; //if any of the actions is not finished, return false.
            }
        }

        return true; //if you looped through all the actions and all of them were finished, then return true;
    }

    @Override
    public void update() {
        for(ActionBase action : actions)
        {
            action.update();
        }        
    }

    @Override
    public void end() {
        for(ActionBase action : actions)
        {
            action.end();
        }        
    }
    
}
