#pragma once

#include <vector>
#include <string>
#include <fstream>

extern int framerate;
extern int sampleRate; // How many times per second the speed and efficiency should be sampled
extern int currentFrame;
extern int secondsPassed;
extern int samplingTime; // The amount of time since the start of the simulation until the sampling is stopped

struct Sample {
	int possibleCollisions;
	int actualCollisions;
	int frame; // The frame the sample occured on
	int seconds; // The amount of whole seconds since the beginning of the simulation
	long executionTime; // The amount of time it took to execute the broad phase (in microseconds)

	inline float GetTime() {
		return (float)seconds + (float)frame / (float)framerate;
	};

	inline double GetEfficiency() {
		return (float)actualCollisions / (float)possibleCollisions;
	}
	
};

void exportSamplesToFile(std::string name);

extern std::vector<Sample> samples;
