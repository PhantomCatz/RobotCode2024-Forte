package frc.robot.Utils.LEDs;

import edu.wpi.first.wpilibj.util.Color;

/**
 * This is a simple interface that can be used to store methods into a variable. 
 * We had to create a custom one since the built-in functional interface, Consumer, does not support varargs (or the ... thing that allows multiple parameters)
 * 
 * Interfaces can essentially be used for two things: serve as a structural basis of a class or used as a collection of methods whose actions are determined on instantiation. 
 */
public interface ColorMethod {
     public void execute(Color... param);
}
