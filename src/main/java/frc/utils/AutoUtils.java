package frc.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;

/*
 * This class contains functions used for autos
 * That's it i guess
 */
public class AutoUtils {
    // A composite auto is represented by a command string
    // each command is separated by a comma
    // e.g. [1, 2, 3] would score game pieces 1, 2 and 3
    // This function takes in that string and returns an array with the separated commands
    // The elements in the array can then be used to get paths and assemble an auto
    // Basically, this removes commas
    public static String[] SeparateCommandString(String _commandString) {
        // make sure the string isn't empty or null or anything before continuing
        if (_commandString.length() == 0) {return null;}

        // defines how many commands are in the string
        // it starts at 1 because there cannot be 0 commands in the string as long as it's not empty
        int commandCount = 1;
        List<Integer> commandIndices = new ArrayList<Integer>();

        // NOTE - to avoid using recursive functions and while loops
        // I'm using two different for loops
        // One for counting the number of commands,
        // And one for getting the commands
        // TODO: make this simpler

        // the first command always starts at index 0
        commandIndices.add(0);

        // loop through the entire string to see how many commands there are
        for (int i = 0; i < _commandString.length(); i++) {
            // if there is a comma at the current position increment the commandCount by 1
            if (_commandString.charAt(i) == ',') {
                commandCount += 1;
                commandIndices.add(i + 1);
            }
        }

        // the array to be returned
        String[] commands = new String[commandCount];

        // populating each command with the provided start indices and end indices that will be found
        for (int i = 0; i < commandIndices.size(); i++) {
            // where does the current command end
            int stopIndex = commandIndices.get(i);

            for (int j = commandIndices.get(i); j < _commandString.length(); j++) {
                // found another comma, so the command is complete
                if (_commandString.charAt(j) == ',') {
                    stopIndex = j - 1;
                    // break the loop and move on to the next command
                    break;
                }
            }

            // define the command as the set of chars from one comma to another
            commands[i] = _commandString.substring(commandIndices.get(i), stopIndex);
        }

        // return all our commands
        return commands;
    }

    // Make an auto using a string[] of commands
    // NOTE - for now each command is just the name of a path,
    // and the paths are assembled into an auto
    // TODO: think more about how commands are going to work, brainstorm with filkin abt this
    public static Command BuildAutoFromCommands(String[] _commands) {
        // Define the auto as a command and add the first followPath command to it

        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile(_commands[0]);
        // Create a path following command using AutoBuilder. This will also trigger event markers.
        Command followCommand = AutoBuilder.followPath(path);

        // loop through the remaining command strings and add them to the auto

        for (int i = 1; i < _commands.length; i++) {
            // get a path file from the name of the command
            path = PathPlannerPath.fromPathFile(_commands[i]);

            // create a followPath command from the path, then add it to the auto using .andThen()
            followCommand = followCommand.andThen(AutoBuilder.followPath(path));
        }

        // return the auto as a command
        return followCommand;
    }
}
