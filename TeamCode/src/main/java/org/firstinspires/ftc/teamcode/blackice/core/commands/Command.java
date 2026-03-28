package org.firstinspires.ftc.teamcode.blackice.core.commands;

public interface Command {
    default void start() {}
    default void update() {}
    boolean isFinished();
    
    default String getName() {
        return "";
    };
    
    static Command singleAction(Runnable action) {
        return new Command() {
            @Override
            public void start() {
                action.run();
            }
            
            @Override
            public boolean isFinished() {
                return true;
            }
            
            @Override
            public String getName() {
                return "singleAction";
            }
        };
    }
    
    default double now() {
        return System.nanoTime() * 1e-9;
    }
    
    default Command untilAllFinish(Command other) {
        Command self = this;
        
        return new Command() {
            @Override
            public void start() {
                self.start();
                other.start();
            }
            
            @Override
            public void update() {
                self.update();
                other.update();
            }
            
            @Override
            public boolean isFinished() {
                return self.isFinished() && other.isFinished();
            }
            
            @Override
            public String getName() {
                return self.getName() + "&" + other.getName();
            }
        };
    }
    
    default Command untilEitherFinishes(Command other) {
        Command self = this;
        
        return new Command() {
            @Override
            public void start() {
                self.start();
                other.start();
            }
            
            @Override
            public void update() {
                self.update();
                other.update();
            }
            
            @Override
            public boolean isFinished() {
                return self.isFinished() || other.isFinished();
            }
            
            @Override
            public String getName() {
                return self.getName() + "|" + other.getName();
            }
        };
    }
    
    // Convenience for multiple commands
    default Command parallel(Command... others) {
        Command result = this;
        for (Command other : others) {
            result = result.untilAllFinish(other);
        }
        return result;
    }
    
    static Command waitSeconds(double seconds) {
        return new Command() {
            private double start;
            
            @Override
            public void start() {
                start = now();
            }
            
            @Override
            public boolean isFinished() {
                return now() - start >= seconds;
            }
            
            @Override
            public String getName() {
                return "wait(" + seconds + "/" + (now() - start) + ")";
            }
        };
    }
    
    default Command withTimeout(double seconds) {
        return this.untilEitherFinishes(waitSeconds(seconds));
    }
}
