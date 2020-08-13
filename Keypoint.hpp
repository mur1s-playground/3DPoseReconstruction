#pragma once

const int KEYPOINT_HEAD = 0;
const int KEYPOINT_THORAX = 1;

const int KEYPOINT_LEFT_SHOULDER = 2;
const int KEYPOINT_LEFT_ARM_UP = 3;
const int KEYPOINT_LEFT_ARM_LO = 4;

const int KEYPOINT_RIGHT_SHOULDER = 5;
const int KEYPOINT_RIGHT_ARM_UP = 6;
const int KEYPOINT_RIGHT_ARM_LO = 7;

const int KEYPOINT_SPINE = 8;

const int KEYPOINT_LEFT_SPINE = 9;
const int KEYPOINT_LEFT_LEG_UP = 10;
const int KEYPOINT_LEFT_LEG_LO = 11;
const int KEYPOINT_LEFT_LEG_LO_F = 24;

const int KEYPOINT_RIGHT_SPINE = 12;
const int KEYPOINT_RIGHT_LEG_UP = 13;
const int KEYPOINT_RIGHT_LEG_LO = 14;
const int KEYPOINT_RIGHT_LEG_LO_F = 21;

const int KEYPOINT_LEFT_FOOT_1 = 22;
const int KEYPOINT_LEFT_FOOT_2 = 23;

const int KEYPOINT_RIGHT_FOOT_1 = 19;
const int KEYPOINT_RIGHT_FOOT_2 = 20;

struct keypoint {
	float x, y, confidence;
};