#ifndef PIXY_LINE_DETECTION_H
#define PIXY_LINE_DETECTION_H

#include <SPI.h>
#include <Pixy2.h>
#include "Arduino.h"
#include <globalVariables.h>
struct xy {
	double x;
	double y;
};

class PixyLineDetect {

public:
	void init();

	// refresh the line vector
	// A low-pass filter may be applied in the future to reduce noise
	void update();

	// return the angle between pixy camera and the line in degree
	// positive: line tilts to the right of camera
	// negative: line tilts to the left of camera
	double getAng() const;
	double getAng(double maxAng) const;

	// return distance of the camera to the line in milimeter
	// return negative number if camera is at left of the line, return postive numebr if camera is at right of the line
	double getOffset() const;
	double getOffset(double maxAng) const;

private:
	// This function takes xy coordiate from the camera view to line plane coordiate. The camera is at the origin. The unit is milimeter
	xy findHorXY(double x, double y) const;
	Pixy2 pixy;
};
#endif