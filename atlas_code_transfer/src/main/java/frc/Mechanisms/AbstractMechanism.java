package frc.Mechanisms;

public abstract class AbstractMechanism implements Runnable {
    private final int delay;
    private Thread thread;

    private boolean runThread;

    public AbstractMechanism(int period) {
        this.delay = period;
    }

    public final void start() {
        if ((thread == null || !thread.isAlive()) && this.delay > 0) {
            thread = new Thread(this);
            runThread = true;
            thread.start();
        }
    }

    public final void kill(){
        runThread = false;
        System.out.println("Thread killed.");
    }

    public abstract void update();
    public abstract void smartDashboard();
    public abstract void smartDashboard_DEBUG();

    public void threadInit(){}

    @Override
    public void run() {
        threadInit();
        while(runThread == true){
            update();
            try{
                Thread.sleep(delay);
            }catch(InterruptedException e){
                System.out.println("Interrupted: " + e.getMessage());
            }
        }
    }
}
