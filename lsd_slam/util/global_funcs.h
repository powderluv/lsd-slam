/**
 * This file is part of LSD-SLAM.
 *
 * Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
 * For more information see <http://vision.in.tum.de/lsdslam>
 *
 * LSD-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LSD-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once
#include <opencv2/core/core.hpp>
#include "util/settings.h"
#include "io_wrapper/timestamped_object.h"
#include "util/sophus_util.h"

namespace lsd_slam {

template<typename T>
class NotifyBuffer;

class Frame;

SE3 SE3CV2Sophus(const cv::Mat& R, const cv::Mat& t);

void printMessageOnCVImage(cv::Mat &image, std::string line1,
		std::string line2);

// reads interpolated element from a uchar* array
// SSE2 optimization possible
inline float getInterpolatedElement(const float* const mat, const float x,
		const float y, const int width) {
	//stats.num_pixelInterpolations++;

	int ix = (int) x;
	int iy = (int) y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx * dy;
	const float* bp = mat + ix + iy * width;

	float res = dxdy * bp[1 + width] + (dy - dxdy) * bp[width]
			+ (dx - dxdy) * bp[1] + (1 - dx - dy + dxdy) * bp[0];

	return res;
}

inline Eigen::Vector3f getInterpolatedElement43(
		const Eigen::Vector4f* const mat, const float x, const float y,
		const int width) {
	int ix = (int) x;
	int iy = (int) y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx * dy;
	const Eigen::Vector4f* bp = mat + ix + iy * width;

	return dxdy * *(const Eigen::Vector3f*) (bp + 1 + width)
			+ (dy - dxdy) * *(const Eigen::Vector3f*) (bp + width)
			+ (dx - dxdy) * *(const Eigen::Vector3f*) (bp + 1)
			+ (1 - dx - dy + dxdy) * *(const Eigen::Vector3f*) (bp);
}

inline Eigen::Vector4f getInterpolatedElement44(
		const Eigen::Vector4f* const mat, const float x, const float y,
		const int width) {
	int ix = (int) x;
	int iy = (int) y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx * dy;
	const Eigen::Vector4f* bp = mat + ix + iy * width;

	return dxdy * *(bp + 1 + width) + (dy - dxdy) * *(bp + width)
			+ (dx - dxdy) * *(bp + 1) + (1 - dx - dy + dxdy) * *(bp);
}

inline Eigen::Vector2f getInterpolatedElement42(
		const Eigen::Vector4f* const mat, const float x, const float y,
		const int width) {
	int ix = (int) x;
	int iy = (int) y;
	float dx = x - ix;
	float dy = y - iy;
	float dxdy = dx * dy;
	const Eigen::Vector4f* bp = mat + ix + iy * width;

	return dxdy * *(const Eigen::Vector2f*) (bp + 1 + width)
			+ (dy - dxdy) * *(const Eigen::Vector2f*) (bp + width)
			+ (dx - dxdy) * *(const Eigen::Vector2f*) (bp + 1)
			+ (1 - dx - dy + dxdy) * *(const Eigen::Vector2f*) (bp);
}
inline void fillCvMat(cv::Mat* mat, cv::Vec3b color) {
	for (int y = 0; y < mat->size().height; y++)
		for (int x = 0; x < mat->size().width; x++)
			mat->at < cv::Vec3b > (y, x) = color;
}

inline void setPixelInCvMat(cv::Mat* mat, cv::Vec3b color, int xx, int yy,
		int lvlFac) {
	for (int x = xx * lvlFac; x < (xx + 1) * lvlFac && x < mat->size().width;
			x++)
		for (int y = yy * lvlFac;
				y < (yy + 1) * lvlFac && y < mat->size().height; y++)
			mat->at < cv::Vec3b > (y, x) = color;
}

inline cv::Vec3b getGrayCvPixel(float val) {
	if (val < 0)
		val = 0;
	if (val > 255)
		val = 255;
	return cv::Vec3b(val, val, val);
}

cv::Mat getDepthRainbowPlot(Frame* kf, int lvl = 0);
cv::Mat getDepthRainbowPlot(const float* idepth, const float* idepthVar,
		const float* gray, int width, int height);
cv::Mat getVarRedGreenPlot(const float* idepthVar, const float* gray, int width,
		int height);
}

/*
 inline std::string currentTimeStamp () {
 return __TIMESTAMP__ ;
 }
 */
// Get current date/time, format is YYYY-MM-DD.HH:mm:ss
inline std::string currentTimeStamp() {
	time_t now = time(0);
	struct tm tstruct;
	char buf[80];
	tstruct = *localtime(&now);
	// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
	// for more information about date/time format
	strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

	return buf;
}

inline char *currentTimeStamp2() {
	char *timestamp = (char *) malloc(sizeof(char) * 23);

	timeval tp;
	gettimeofday(&tp, 0);
	time_t curtime = tp.tv_sec;
	tm *t = localtime(&curtime);
	sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d.%03ld", t->tm_year + 1900, t->tm_mon,
			t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec, tp.tv_usec / 1000);
	return timestamp;
}

/**
 * message - message to be logged
 * level - log level (0 - INFO, 1 - WARN, 2 - DEBUG, 3 - ERROR, 4 - FATAL
 */
inline void log(std::string message, int level) {
	std::string logLevel = "INFO";
	switch (level) {
	case 0:
		logLevel = "INFO";
		break;
	case 1:
		logLevel = "WARN";
		break;
	case 2:
		logLevel = "DEBUG";
		break;
	case 3:
		logLevel = "ERROR";
		break;
	case 4:
		logLevel = "FATAL";
		break;
	}

	char *str;

	printf("%s - %s: %s\n", currentTimeStamp2(), logLevel.c_str(),
			message.c_str());

}

