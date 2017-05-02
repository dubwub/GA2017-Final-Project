# Darwin Ding's Final Project (Rhythm Game Framework)

Presentation:

https://docs.google.com/presentation/d/1oZxYPMvms2N-Q0w2Twk_rQC29WI5iS5iQEKqtYV7c9w/edit?usp=sharing

Main features are in:

rhythm/ga_rhythm_manager.h and .cpp

How to use:

1. Run ga.exe

2. Press c to turn on calibration mode, and press spacebar when you think the beat is (if you're off here you'll be off in general)

3. Press c again to turn off calib mode, and resume pressing spacebar on the beat
	-> Big explosions = excellent hit, small explosions = nice hit

4. Optionally recompile with the PLAYBACK_MODE flag as true in ga_rhythm_manager.h to instead play beatmap.txt

5. If you would like to record your own beatmap, delete beatmap.txt, then run the program and press r to put it in recording mode and press spacebar whenever (note that you cannot record in playback_mode either. i know this is a little tedious, but it's more proof of concept)

Note that the beatmap functionality is a little bit hacked together and might have a few bugs here and there, but is generally correct.

Please feel free to email me (dingd@rpi.edu) if there are any issues.