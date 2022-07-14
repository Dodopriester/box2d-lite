#include "box2d-lite/Sample.h"

int framerate = 60;
int sampleRate = 2; // How many times per second the speed and efficiency should be sampled
int currentFrame = 0;
int secondsPassed = 0;
int samplingTime = 10;

std::string sample_export_filename = "test";

std::vector<Sample> samples;

void exportSamplesToFile(std::string name)
{
	std::ofstream file;
	file.open(name + ".csv");
	std::string columnHeaders = "Time (s),Seconds,Frame,Execution Time (microseconds),Possible Collisions,Actual Collisions,Efficiency\n";

	file << columnHeaders;

	for (int i = 0; i < samples.size(); i++) {
		Sample sample = samples[i];
		
		std::string row = std::to_string(sample.GetTime()) + "," + std::to_string(sample.seconds) + "," + std::to_string(sample.frame)
			+ "," + std::to_string(sample.executionTime) + "," + std::to_string(sample.possibleCollisions) + ","
			+ std::to_string(sample.actualCollisions) + "," + std::to_string(sample.efficiency) + "\n";

		file << row;
	}

	file.close();

}
