# BadRobot2025

## IMPORTANT
## After every meeting it is important to write a few things on the whiteboard in the programming room. 

1. A detailed description of what code is currently deployed on the robot. (including if it works correctly)
2. A progress report of the current relevant issues on the project board
3. What was completed during the meeting

## Style guide
### we use the standard java style for the most part.
- COMMENTS ARE IMPORTANT! Please provide a serious comment about what your code does
- CONSTANTS ARE IMPORTANT! don't just throw raw numbers around unless it will never change, like a zero for instance. Create all constants in the constants file.
- Name your commits seriously so people can read it and imediately tell what changed
- Tab is the equivilent of two spaces
- variables and functions are camel case
- Classes and file names are pascal case
- Constants are camel case and start with a "k". It is important they are put in a class in the constants file
- Commands are put in the commands folder. Command file and class names end with "Command"
- Subsystems are put in the subsystems folder. Subsystem file and class names end with "Subsystem"
- Any extra helper class should be put in the Utils folder and does not require a suffix

## Contributing guide
First off, please create a new branch on the issue you are working on. Please ask someone if you don't know what this means. 
Commit every so often after important changes. Please update the project board as you go. When your code is ready to be reviewed, make a pull request into main. 
Again, if you don't know how to do this ask someone. After someone reviews your code it will be merged into main. DO NOT PULL REQUEST BEFORE TESTING!

## Guide for reviewing code
Look over the code they changed. Please take this step seriously. If there is a section that goes against the style guide, 
for example if there are no comments or the commit names aren't intuitive, deny the request and comment what they need to fix.
If everything looks good, please accept the pull request and merge the pull request in.
