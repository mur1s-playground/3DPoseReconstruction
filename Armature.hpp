#pragma once

#include <string>
#include <map>
#include <vector>

#include "Keypoint.hpp"

using namespace std;

struct vector2f {
	float x, y;
};

struct vector3f {
	float x, y, z;
};

struct bone {
	vector3f head;
	vector3f tail;

	vector2f ball_coords;

	vector2f ball_coords_theta_constr;
	vector2f ball_coords_phi_constr;
};

enum armature_transformation_type {
	ATT_TRANSLATE_ALL,
	ATT_TRANSLATE_BONE,
	ATT_ROTATE_ALL,
	ATT_ROTATE_BONE
};

struct armature_transformation {
	enum armature_transformation_type	att;
	struct vector3f						att_param;
};

struct armature {
	struct vector3f position;
	struct vector2f	ball_coords;
	float			scale;

	struct bone		head;

	struct bone		thorax;

	struct bone		shoulder_r;
	struct bone		arm_up_r;
	struct bone		arm_lo_r;

	struct bone		shoulder_l;
	struct bone		arm_up_l;
	struct bone		arm_lo_l;

	struct bone		spine;

	struct bone		spine_r;
	struct bone		leg_up_r;
	struct bone		leg_lo_r;
	struct bone		foot_r;

	struct bone		spine_l;
	struct bone		leg_up_l;
	struct bone		leg_lo_l;
	struct bone		foot_l;
};

extern vector<string>				armature_bones;

extern map<string, vector<string>>	armature_bone_dependencies;

void armature_base_init(struct armature* a, float head_len, float spine_len, float thorax_len, float shoulder_radius, float arm_up_len, float arm_lo_len, float spine_radius, float leg_up_len, float leg_lo_len, float foot_len);
void armature_proj_from_keypoints(struct armature* a, vector<struct keypoint>& kps);
void armature_print(struct armature* a, bool abs);

struct armature armature_combine(struct armature* a, struct armature* b, float mutation_rate);
void armature_fit(struct armature* base, struct armature* onto_proj);
void armature_translate_all_bones(struct armature* a, struct vector3f translation);

float armature_bone_length(struct bone* b);
struct bone* armature_bone_get_from_name(struct armature* a, string bone_name);
struct bone armature_bone_get_abs_pos(struct armature* a, string bone_name);