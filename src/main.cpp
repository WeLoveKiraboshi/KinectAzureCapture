#include "../include/captureRGBD.h"

int imageWidth = 640;
int imageHeight = 480;


void parseArgument(char* arg)
{
	int option;
	float foption;
	char buf[1000];

	if (1 == sscanf(arg, "width=%d", &option)) {
		imageWidth = option;
		printf("width=%d\n", imageWidth);
		return;
	}

	if (1 == sscanf(arg, "height=%d", &option)) {
		imageHeight = option;
		printf("height=%d\n", imageHeight);
		return;
	}

	printf("could not parse argument \"%s\"!!!!\n", arg);

}


int main(int argc, char** argv)
{
	
	for (int i = 1; i<argc; i++)
		parseArgument(argv[i]);


	Capture *capture = new Capture(imageWidth, imageHeight);

	capture->sysLoop();

	return 0;
}
