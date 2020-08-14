#include "Armature.hpp"

#include <vector>
#include <list>

#define M_PI 3.14159265358979323846f

#include "3DPoseReconstruction.hpp"

#define rand() myrand()

using namespace std;

#include <iostream>
#include <fstream>
#include <sstream>

vector<string>				armature_bones = {
	"spine"			,
	"spine_l"		,
	"spine_r"		,
	"leg_up_l"		,
	"leg_up_r"		,
	"leg_lo_l"		,
	"leg_lo_r"		,
	"foot_l"		,
	"foot_r"		,
	"thorax"		,
	"shoulder_l"	,
	"shoulder_r"	,
	"arm_up_l"		,
	"arm_up_r"		,
	"arm_lo_l"		,
	"arm_lo_r"		,
	"head"
};

vector<map<string, struct armature_genetic_toggle>>			armature_gen_sets_stages = {
	{	//Stage 1
		{"spine"	, {true, true, true} }		, {"spine_l"	, {true, true, false}}		, {"spine_r"	, {true, true, false}},

		{"leg_up_l"	, {false, false, false}}	, {"leg_up_r"	, {false, false, false}}	, {"leg_lo_l"	, {false, false, false}}	, {"leg_lo_r", {false, false, false}},  
		{"foot_l"	, {false, false, false}}	, {"foot_r"		, {false, false, false}}	,

		{"thorax"	, {true, true, true} }		, {"shoulder_l"	, {true, true, true}}		, {"shoulder_r"	, {true, true, true}}		,

		{"arm_up_l"	, {false, false, false}}	, {"arm_up_r"	, {false, false, false}}	, {"arm_lo_l"	, {false, false, false}}	, {"arm_lo_r", {false, false, false}},

		{"head"		, {true, true, false}}
	},
	{	//Stage 2
		{"spine"	, {false, false, false} }	, {"spine_l"	, {false, false, false}}	, {"spine_r"	, {false, false, false}}	,

		{"leg_up_l"	, {true, true, true}}		, {"leg_up_r"	, {true, true, true}}		, {"leg_lo_l"	, {true, true, true}}		, {"leg_lo_r", {true, true, true}},
		{"foot_l"	, {true, true, true}}		, {"foot_r"		, {true, true, true}}		,

		{"thorax"	, {false, false, false}}	, {"shoulder_l"	, {false, false, false}}	, {"shoulder_r"	, {false, false, false}}	,

		{"arm_up_l"	, {true, true, true}}		, {"arm_up_r"	, {true, true, true}}		, {"arm_lo_l"	, {true, true, true}}		, {"arm_lo_r", {true, true, true}},

		{"head"		, {false, false, false}}
	},
};

map<string, vector<string>>	armature_bone_dependencies = {
	{"spine"		, {}},
	{"spine_l"		, {}},
	{"spine_r"		, {}},
	{"leg_up_l"		, {"spine_l"}},
	{"leg_up_r"		, {"spine_r"}},
	{"leg_lo_l"		, {"spine_l", "leg_up_l"}},
	{"leg_lo_r"		, {"spine_r", "leg_up_r"}},
	{"foot_l"		, {"spine_l", "leg_up_l", "leg_lo_l"}},
	{"foot_r"		, {"spine_r", "leg_up_r", "leg_lo_r"}},
	{"thorax"		, {"spine"}},
	{"shoulder_l"	, {"spine", "thorax"}},
	{"shoulder_r"	, {"spine", "thorax"}},
	{"arm_up_l"		, {"spine", "thorax", "shoulder_l"}},
	{"arm_up_r"		, {"spine", "thorax", "shoulder_r"}},
	{"arm_lo_l"		, {"spine", "thorax", "shoulder_l", "arm_up_l"}},
	{"arm_lo_r"		, {"spine", "thorax", "shoulder_r", "arm_up_r"}},
	{"head"			, {"spine", "thorax"}}
};

//tail fixed, head moves
void armature_base_init(struct armature* a, float head_len, float spine_len, float thorax_len, float shoulder_radius, float arm_up_len, float arm_lo_len, float spine_radius, float leg_up_len, float leg_lo_len, float foot_len) {
	struct vector2f foot_theta_constr		= { -M_PI/4.0f			, -M_PI/4.0f + M_PI * 5.0/6.0f };
	struct vector2f foot_phi_constr			= { -M_PI/8.0f			, M_PI/8.0f };
	struct vector2f	foot_psi_constr			= { -M_PI/4.0f			, M_PI/4.0f };

	struct vector2f leg_lo_theta_constr		= { 0.0f				, M_PI * 9.0/10.0f };
	struct vector2f leg_lo_phi_constr		= { -M_PI/32.0f			, M_PI/32.0f };
	struct vector2f leg_lo_psi_constr		= { 0.0f				, 0.0f };

	//struct vector2f leg_up_theta_constr		= { -M_PI/2.0f			, M_PI/2.0f};
	//struct vector2f leg_up_phi_constr		= { -M_PI/2.0f			, M_PI/2.0f};

	struct vector2f leg_up_theta_constr		= { -M_PI			, M_PI };
	struct vector2f leg_up_phi_constr		= { -M_PI			, M_PI };
	struct vector2f leg_up_psi_constr		= { 0.0f			, M_PI };

	struct vector2f spine_s_theta_constr	= { -M_PI/64.0f			, M_PI/64.0f };
	struct vector2f spine_s_phi_constr		= { 0.0f				, 0.0f };
	struct vector2f spine_s_psi_constr		= { 0.0f				, 0.0f };

	struct vector2f shoulder_theta_constr	= { -M_PI/4.0f			, M_PI/4.0f	};
	struct vector2f shoulder_phi_constr		= { -M_PI/4.0f			, M_PI/4.0f };
	struct vector2f shoulder_psi_constr		= { 0.0f				, 0.0f };

	struct vector2f arm_up_theta_constr		= { -M_PI/2.0f			, M_PI/2.0f };
	struct vector2f arm_up_phi_constr		= { -M_PI * 9.0f/10.0f	, M_PI * 9.0f/10.0f };
	struct vector2f arm_up_psi_constr		= { -M_PI / 2.0f		, M_PI };

	struct vector2f arm_lo_theta_constr		= { 0.0f				, M_PI * 9.0f / 10.0f };
	struct vector2f arm_lo_phi_constr		= { -M_PI/32.0f			, M_PI/32.0f };
	struct vector2f arm_lo_psi_constr		= { 0.0f				, M_PI };

	a->scale = 1.0f;

	a->foot_l.tail = { -spine_radius, 0.0		, 0.0 };
	a->foot_l.head = { -spine_radius, foot_len	, 0.0 };
	a->foot_l.ball_coords_theta_constr = foot_theta_constr;
	a->foot_l.ball_coords_phi_constr = foot_phi_constr;
	a->foot_l.ball_coords_psi_constr = foot_psi_constr;
	a->foot_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->leg_lo_l.head = { a->foot_l.tail.x	, a->foot_l.tail.y	, a->foot_l.tail.z			  };
	a->leg_lo_l.tail = { a->leg_lo_l.head.x	, a->leg_lo_l.head.y, a->leg_lo_l.head.z+leg_lo_len };
	a->leg_lo_l.ball_coords_theta_constr = leg_lo_theta_constr;
	a->leg_lo_l.ball_coords_phi_constr = leg_lo_phi_constr;
	a->leg_lo_l.ball_coords_psi_constr = leg_lo_psi_constr;
	a->leg_lo_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->leg_up_l.head = { a->leg_lo_l.tail.x, a->leg_lo_l.tail.y , a->leg_lo_l.tail.z };
	a->leg_up_l.tail = { a->leg_up_l.head.x, a->leg_up_l.head.y , a->leg_up_l.head.z+leg_up_len };
	a->leg_up_l.ball_coords_theta_constr = leg_up_theta_constr;
	a->leg_up_l.ball_coords_phi_constr = leg_up_phi_constr;
	a->leg_up_l.ball_coords_psi_constr = leg_up_psi_constr;
	a->leg_up_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->spine_l.head = { a->leg_up_l.tail.x, a->leg_up_l.tail.y , a->leg_up_l.tail.z };
	a->spine_l.tail = { a->spine_l.head.x+spine_radius, a->spine_l.head.y , a->spine_l.head.z };
	a->spine_l.ball_coords_theta_constr = spine_s_theta_constr;
	a->spine_l.ball_coords_phi_constr = spine_s_phi_constr;
	a->spine_l.ball_coords_psi_constr = spine_s_psi_constr;
	a->spine_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->foot_r.tail = { spine_radius	, 0.0		, 0.0 };
	a->foot_r.head = { spine_radius	, foot_len	, 0.0 };
	a->foot_r.ball_coords_theta_constr = foot_theta_constr;
	a->foot_r.ball_coords_phi_constr = foot_phi_constr;
	a->foot_r.ball_coords_psi_constr = { -foot_psi_constr.y, -foot_psi_constr.x };
	a->foot_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->leg_lo_r.head = { a->foot_r.tail.x	, a->foot_r.tail.y	, a->foot_r.tail.z };
	a->leg_lo_r.tail = { a->leg_lo_r.head.x	, a->leg_lo_r.head.y, a->leg_lo_r.head.z + leg_lo_len };
	a->leg_lo_r.ball_coords_theta_constr = leg_lo_theta_constr;
	a->leg_lo_r.ball_coords_phi_constr = leg_lo_phi_constr;
	a->leg_lo_r.ball_coords_psi_constr = { -leg_lo_psi_constr.y, -leg_lo_psi_constr.x };
	a->leg_lo_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->leg_up_r.head = { a->leg_lo_r.tail.x, a->leg_lo_r.tail.y , a->leg_lo_r.tail.z };
	a->leg_up_r.tail = { a->leg_up_r.head.x, a->leg_up_r.head.y , a->leg_up_r.head.z + leg_up_len };
	a->leg_up_r.ball_coords_theta_constr = leg_up_theta_constr;
	a->leg_up_r.ball_coords_phi_constr = leg_up_phi_constr;
	a->leg_up_r.ball_coords_psi_constr = { -leg_up_psi_constr.y, -leg_lo_psi_constr.x };
	a->leg_up_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->spine_r.head = { a->leg_up_r.tail.x, a->leg_up_r.tail.y , a->leg_up_r.tail.z };
	a->spine_r.tail = { a->spine_r.head.x-spine_radius, a->spine_r.head.y , a->spine_r.head.z };
	a->spine_r.ball_coords_theta_constr = spine_s_theta_constr;
	a->spine_r.ball_coords_phi_constr = spine_s_phi_constr;
	a->spine_r.ball_coords_psi_constr = { -spine_s_psi_constr.y, -spine_s_psi_constr.x };
	a->spine_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->spine.tail = { a->spine_r.tail.x, a->spine_r.tail.y, a->spine_r.tail.z };
	a->spine.head = { a->spine.tail.x, a->spine.tail.y, a->spine.tail.z+spine_len };
	a->spine.ball_coords_theta_constr = { -M_PI/16.0f, M_PI };
	a->spine.ball_coords_phi_constr = { -M_PI, M_PI };
	a->spine.ball_coords_psi_constr = { 0.0f, 0.0f };
	a->spine.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->thorax.tail = { a->spine.head.x, a->spine.head.y, a->spine.head.z };
	a->thorax.head = { a->spine.head.x, a->spine.head.y, a->spine.head.z+thorax_len};
	a->thorax.ball_coords_theta_constr = { 0.0f, 0.0f };
	a->thorax.ball_coords_phi_constr = { 0.0f, 0.0f };
	a->thorax.ball_coords_psi_constr = { 0.0f, 0.0f };
	a->thorax.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->shoulder_l.tail = { a->thorax.head.x, a->thorax.head.y, a->thorax.head.z };
	a->shoulder_l.head = { a->shoulder_l.tail.x-shoulder_radius, a->shoulder_l.tail.y, a->shoulder_l.tail.z };
	a->shoulder_l.ball_coords_theta_constr = shoulder_theta_constr;
	a->shoulder_l.ball_coords_phi_constr = shoulder_phi_constr;
	a->shoulder_l.ball_coords_psi_constr = shoulder_psi_constr;
	a->shoulder_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->arm_up_l.tail = { a->shoulder_l.head.x, a->shoulder_l.head.y , a->shoulder_l.head.z };
	a->arm_up_l.head = { a->arm_up_l.tail.x, a->arm_up_l.tail.y, a->arm_up_l.tail.z-arm_up_len};
	a->arm_up_l.ball_coords_theta_constr = arm_up_theta_constr;
	a->arm_up_l.ball_coords_phi_constr = arm_up_phi_constr;
	a->arm_up_l.ball_coords_psi_constr = arm_up_psi_constr;
	a->arm_up_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->arm_lo_l.tail = { a->arm_up_l.head.x, a->arm_up_l.head.y, a->arm_up_l.head.z };
	a->arm_lo_l.head = { a->arm_lo_l.tail.x, a->arm_lo_l.tail.y, a->arm_lo_l.tail.z-arm_lo_len};
	a->arm_lo_l.ball_coords_theta_constr = arm_lo_theta_constr;
	a->arm_lo_l.ball_coords_phi_constr = arm_lo_phi_constr;
	a->arm_lo_l.ball_coords_psi_constr = arm_lo_psi_constr;
	a->arm_lo_l.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->shoulder_r.tail = { a->thorax.head.x, a->thorax.head.y, a->thorax.head.z };
	a->shoulder_r.head = { a->shoulder_r.tail.x + shoulder_radius, a->shoulder_r.tail.y, a->shoulder_r.tail.z };
	a->shoulder_r.ball_coords_theta_constr = shoulder_theta_constr;
	a->shoulder_r.ball_coords_phi_constr = shoulder_phi_constr;
	a->shoulder_r.ball_coords_psi_constr = { -shoulder_psi_constr.y, -shoulder_psi_constr.x };
	a->shoulder_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->arm_up_r.tail = { a->shoulder_r.head.x, a->shoulder_r.head.y , a->shoulder_r.head.z };
	a->arm_up_r.head = { a->arm_up_r.tail.x, a->arm_up_r.tail.y, a->arm_up_r.tail.z - arm_up_len };
	a->arm_up_r.ball_coords_theta_constr = arm_up_theta_constr;
	a->arm_up_r.ball_coords_phi_constr = arm_up_phi_constr;
	a->arm_up_r.ball_coords_psi_constr = { -arm_up_psi_constr.y, -arm_up_psi_constr.x };
	a->arm_up_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->arm_lo_r.tail = { a->arm_up_r.head.x, a->arm_up_r.head.y, a->arm_up_r.head.z };
	a->arm_lo_r.head = { a->arm_lo_r.tail.x, a->arm_lo_r.tail.y, a->arm_lo_r.tail.z - arm_lo_len };
	a->arm_lo_r.ball_coords_theta_constr = arm_lo_theta_constr;
	a->arm_lo_r.ball_coords_phi_constr = arm_lo_phi_constr;
	a->arm_lo_r.ball_coords = { 0.0f, 0.0f, 0.0f };

	a->head.tail = { a->thorax.head.x, a->thorax.head.y, a->thorax.head.z };
	a->head.head = { a->head.tail.x, a->head.tail.y, a->head.tail.z+head_len};
	a->head.ball_coords_theta_constr = {-M_PI/4.0f, M_PI/4.0f };
	a->head.ball_coords_phi_constr = {-M_PI/2.0f, M_PI/2.0f};
	a->head.ball_coords_psi_constr = {-M_PI, M_PI };
	a->head.ball_coords = { 0.0f, 0.0f, 0.0f };
}

void armature_proj_from_keypoints(struct armature *a, vector<struct keypoint> &kps) {
	a->scale = 1.0f;

	a->head.tail		= { kps[KEYPOINT_THORAX].x, 0.0f, kps[KEYPOINT_THORAX].y };
	a->head.head		= { kps[KEYPOINT_HEAD].x, 0.0f, kps[KEYPOINT_HEAD].y };

	a->thorax.tail		= { (kps[KEYPOINT_SPINE].x + kps[KEYPOINT_THORAX].x) * 0.5f, 0.0f, (kps[KEYPOINT_SPINE].y + kps[KEYPOINT_THORAX].y) * 0.5f};
	a->thorax.head		= { kps[KEYPOINT_THORAX].x, 0.0f, kps[KEYPOINT_THORAX].y};

	a->shoulder_l.tail	= { kps[KEYPOINT_THORAX].x, 0.0f, kps[KEYPOINT_THORAX].y };
	a->shoulder_l.head	= { kps[KEYPOINT_LEFT_SHOULDER].x, 0.0f, kps[KEYPOINT_LEFT_SHOULDER].y };

	a->shoulder_r.tail	= { kps[KEYPOINT_THORAX].x, 0.0f, kps[KEYPOINT_THORAX].y };
	a->shoulder_r.head	= { kps[KEYPOINT_RIGHT_SHOULDER].x, 0.0f,	kps[KEYPOINT_RIGHT_SHOULDER].y };

	a->spine.tail		= { kps[KEYPOINT_SPINE].x, 0.0f, kps[KEYPOINT_SPINE].y };
	a->spine.head		= { (kps[KEYPOINT_SPINE].x + kps[KEYPOINT_THORAX].x) * 0.5f, 0.0f, (kps[KEYPOINT_SPINE].y + kps[KEYPOINT_THORAX].y) * 0.5f };

	a->spine_l.tail		= { kps[KEYPOINT_SPINE].x, 0.0f, kps[KEYPOINT_SPINE].y };
	a->spine_l.head		= {	kps[KEYPOINT_LEFT_SPINE].x, 0.0f, kps[KEYPOINT_LEFT_SPINE].y };

	a->spine_r.tail		= { kps[KEYPOINT_SPINE].x, 0.0f, kps[KEYPOINT_SPINE].y };
	a->spine_r.head		= { kps[KEYPOINT_RIGHT_SPINE].x, 0.0f, kps[KEYPOINT_RIGHT_SPINE].y };

	a->arm_up_l.tail	= { kps[KEYPOINT_LEFT_SHOULDER].x, 0.0f, kps[KEYPOINT_LEFT_SHOULDER].y };
	a->arm_up_l.head	= { kps[KEYPOINT_LEFT_ARM_UP].x, 0.0f, kps[KEYPOINT_LEFT_ARM_UP].y };

	a->arm_up_r.tail	= { kps[KEYPOINT_RIGHT_SHOULDER].x, 0.0f,	kps[KEYPOINT_RIGHT_SHOULDER].y };
	a->arm_up_r.head	= { kps[KEYPOINT_RIGHT_ARM_UP].x, 0.0f, kps[KEYPOINT_RIGHT_ARM_UP].y };

	a->arm_lo_l.tail	= { kps[KEYPOINT_LEFT_ARM_UP].x, 0.0f, kps[KEYPOINT_LEFT_ARM_UP].y };
	a->arm_lo_l.head	= { kps[KEYPOINT_LEFT_ARM_LO].x, 0.0f, kps[KEYPOINT_LEFT_ARM_LO].y };

	a->arm_lo_r.tail	= { kps[KEYPOINT_RIGHT_ARM_UP].x, 0.0f, kps[KEYPOINT_RIGHT_ARM_UP].y };
	a->arm_lo_r.head	= { kps[KEYPOINT_RIGHT_ARM_LO].x, 0.0f, kps[KEYPOINT_RIGHT_ARM_LO].y };

	a->leg_up_l.tail	= { kps[KEYPOINT_LEFT_SPINE].x, 0.0f, kps[KEYPOINT_LEFT_SPINE].y };
	a->leg_up_l.head	= { kps[KEYPOINT_LEFT_LEG_UP].x, 0.0f, kps[KEYPOINT_LEFT_LEG_UP].y };

	a->leg_up_r.tail	= { kps[KEYPOINT_RIGHT_SPINE].x, 0.0f, kps[KEYPOINT_RIGHT_SPINE].y };
	a->leg_up_r.head	= { kps[KEYPOINT_RIGHT_LEG_UP].x, 0.0f, kps[KEYPOINT_RIGHT_LEG_UP].y };

	a->leg_lo_l.tail	= { kps[KEYPOINT_LEFT_LEG_UP].x, 0.0f, kps[KEYPOINT_LEFT_LEG_UP].y };
	a->leg_lo_l.head	= { (kps[KEYPOINT_LEFT_LEG_LO].x + kps[KEYPOINT_LEFT_LEG_LO_F].x) * 0.5f, 0.0f, (kps[KEYPOINT_LEFT_LEG_LO].y + kps[KEYPOINT_LEFT_LEG_LO_F].y) * 0.5f };

	a->leg_lo_r.tail	= { kps[KEYPOINT_RIGHT_LEG_UP].x, 0.0f, kps[KEYPOINT_RIGHT_LEG_UP].y };
	a->leg_lo_r.head	= { (kps[KEYPOINT_RIGHT_LEG_LO].x + kps[KEYPOINT_RIGHT_LEG_LO_F].x) * 0.5f, 0.0f, (kps[KEYPOINT_RIGHT_LEG_LO].y + kps[KEYPOINT_RIGHT_LEG_LO_F].y) * 0.5f };

	a->foot_l.tail		= { a->leg_lo_l.head.x, 0.0f, a->leg_lo_l.head.z};
	a->foot_l.head		= { (kps[KEYPOINT_LEFT_FOOT_1].x + kps[KEYPOINT_LEFT_FOOT_2].x) * 0.5f, 0.0f, (kps[KEYPOINT_LEFT_FOOT_1].y + kps[KEYPOINT_LEFT_FOOT_2].y) * 0.5f };

	a->foot_r.tail		= { a->leg_lo_r.head.x, 0.0f, a->leg_lo_r.head.z };
	a->foot_r.head		= { (kps[KEYPOINT_RIGHT_FOOT_1].x + kps[KEYPOINT_RIGHT_FOOT_2].x) * 0.5f, 0.0f, (kps[KEYPOINT_RIGHT_FOOT_1].y + kps[KEYPOINT_RIGHT_FOOT_2].y) * 0.5f };

	a->foot_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->leg_lo_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->leg_up_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->spine_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->foot_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->leg_lo_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->leg_up_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->spine_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->spine.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->thorax.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->shoulder_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->arm_up_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->arm_lo_l.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->shoulder_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->arm_up_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->arm_up_r.ball_coords = { 0.0f, 0.0f, 0.0f };
	a->head.ball_coords = { 0.0f, 0.0f, 0.0f };
}

bool armature_gen_sets_bone_has_enabled_param(string bone_name, int stage) {
	if (armature_gen_sets_stages[stage][bone_name].x || armature_gen_sets_stages[stage][bone_name].y || armature_gen_sets_stages[stage][bone_name].z) return true;
	return false;
}

float armature_fitness(struct armature *base, struct armature *onto_proj, int stage) {
	float err = 0.0f;
	for (int i = 0; i < armature_bones.size(); i++) {
		if (armature_gen_sets_bone_has_enabled_param(armature_bones[i], stage)) {
			struct bone current_bone = armature_bone_get_abs_pos(base, armature_bones[i]);
			struct bone* proj_bone = armature_bone_get_from_name(onto_proj, armature_bones[i]);

			struct vector2f diff_t = { current_bone.tail.x - proj_bone->tail.x, current_bone.tail.z - proj_bone->tail.z };
			float			norm_t = sqrtf(diff_t.x * diff_t.x + diff_t.y * diff_t.y);

			struct vector2f diff_h = { current_bone.head.x - proj_bone->head.x, current_bone.head.z - proj_bone->head.z };
			float			norm_h = sqrtf(diff_h.x * diff_h.x + diff_h.y * diff_h.y);

			err += (norm_t + norm_h);
		}
	}
	return err;
}

void armature_set_random_genetic(struct armature *a, int stage) {
	if (stage == 0) {
		a->scale = 1.0 - (rand() / (float)RAND_MAX) * 0.5;
	}
	for (int i = 0; i < armature_bones.size(); i++) {
		struct bone* current = armature_bone_get_from_name(a, armature_bones[i]);
		if (armature_gen_sets_stages[stage][armature_bones[i]].x) {
			current->ball_coords.x = current->ball_coords_theta_constr.x + (rand() / (float)RAND_MAX) * (current->ball_coords_theta_constr.y - current->ball_coords_theta_constr.x);
		}
		if (armature_gen_sets_stages[stage][armature_bones[i]].y) {
			current->ball_coords.y = current->ball_coords_phi_constr.x + (rand() / (float)RAND_MAX) * (current->ball_coords_phi_constr.y - current->ball_coords_phi_constr.x);
		}
		if (armature_gen_sets_stages[stage][armature_bones[i]].z) {
			current->ball_coords.z = current->ball_coords_psi_constr.x + (rand() / (float)RAND_MAX) * (current->ball_coords_psi_constr.y - current->ball_coords_psi_constr.x);
		}
	}
}

struct armature armature_combine(struct armature* a, struct armature* b, float mutation_rate, int stage) {
	int splt = floor(rand() / (float)RAND_MAX * (armature_bones.size()-1));
	struct armature result = *a;

	if (stage == 0) {
		float rand_comb = rand() / (float)RAND_MAX;
		if (rand_comb < mutation_rate) {
			result.scale = 1.0 - (rand() / (float)RAND_MAX) * 0.5;
		} else if (rand_comb < (1 - mutation_rate) / 3.0f) {
			result.scale = (a->scale + b->scale) * 0.5;
		} else if (rand_comb < 2 * (1 - mutation_rate) / 3.0f) {
			result.scale = a->scale;
		} else {
			result.scale = b->scale;
		}
	}
	
	for (int i = 0; i < armature_bones.size(); i++) {
		struct bone* to_set = armature_bone_get_from_name(&result, armature_bones[i]);

		struct bone* from_a = armature_bone_get_from_name(a, armature_bones[i]);
		struct bone* from_b = armature_bone_get_from_name(b, armature_bones[i]);

		float rand_comb = rand() / (float)RAND_MAX;

		if (rand_comb < mutation_rate) {
			to_set->ball_coords.x = from_a->ball_coords_theta_constr.x + (rand() / (float)RAND_MAX) * (from_a->ball_coords_theta_constr.y - from_a->ball_coords_theta_constr.x);
			to_set->ball_coords.y = from_a->ball_coords_phi_constr.y + (rand() / (float)RAND_MAX) * (from_a->ball_coords_phi_constr.y - from_a->ball_coords_phi_constr.y);
			to_set->ball_coords.z = from_a->ball_coords_psi_constr.y + (rand() / (float)RAND_MAX) * (from_a->ball_coords_psi_constr.y - from_a->ball_coords_psi_constr.y);
		} else if (rand_comb < (1 - mutation_rate) / 3.0f) {
			to_set->ball_coords.x = (from_a->ball_coords.x + from_b->ball_coords.x) * 0.5;
			to_set->ball_coords.y = (from_a->ball_coords.y + from_b->ball_coords.y) * 0.5;
			to_set->ball_coords.z = (from_a->ball_coords.z + from_b->ball_coords.z) * 0.5;
		} else if (rand_comb < 2 * (1 - mutation_rate) / 3.0f) {
			to_set->ball_coords.x = from_a->ball_coords.x;
			to_set->ball_coords.y = from_a->ball_coords.y;
			to_set->ball_coords.z = from_a->ball_coords.z;
		} else {
			to_set->ball_coords.x = from_b->ball_coords.x;
			to_set->ball_coords.y = from_b->ball_coords.y;
			to_set->ball_coords.z = from_b->ball_coords.z;
		}

		if (!armature_gen_sets_stages[stage][armature_bones[i]].x) {
			to_set->ball_coords.x = from_a->ball_coords.x;
		}
		if (!armature_gen_sets_stages[stage][armature_bones[i]].y) {
			to_set->ball_coords.y = from_a->ball_coords.y;
		}
		if (!armature_gen_sets_stages[stage][armature_bones[i]].z) {
			to_set->ball_coords.z = from_a->ball_coords.z;
		}
	}
	return result;
}

int w_idx = 0;

void armature_fit(struct armature* base, struct armature* onto_proj) {
	struct vector3f spine_translation_param = { base->spine.tail.x - onto_proj->spine.tail.x, base->spine.tail.y - onto_proj->spine.tail.y, base->spine.tail.z - onto_proj->spine.tail.z };
	armature_translate_all_bones(onto_proj, spine_translation_param);

	vector<struct armature> armature_population;
	vector<float>			armature_fitness_f;
	float					fitness_max = 0.0f;
	float					fitness_min = FLT_MAX;
	int						fitness_min_idx = -1;

	struct armature			armature_best;
	float					fitness_best = FLT_MAX;

	int population_size = 128;
	float mutation_rate = 0.05;
	int generations = 100;

	for (int s = 0; s < armature_gen_sets_stages.size(); s++) {
		float					tmp_fitness_max = fitness_max;
		float					tmp_fitness_min = fitness_min;

		fitness_max = 0.0f;
		fitness_min = FLT_MAX;
		fitness_min_idx = -1;
		fitness_best = FLT_MAX;
		
		vector<float>			tmp_fitness = armature_fitness_f;
		vector<struct armature> tmp_pop = armature_population;
		
		armature_population.clear();
		armature_fitness_f.clear();

		if (tmp_pop.size() > 0) {
			armature_population.push_back(armature_best);
			armature_fitness_f.push_back(armature_fitness(&armature_best, onto_proj, s));
			fitness_best = armature_fitness_f[0];
			fitness_min = armature_fitness_f[0];
			fitness_max = armature_fitness_f[0];
			fitness_min_idx = 0;
		}

		for (int i = 0; i < population_size; i++) {
			struct armature current = *base;
			if (tmp_pop.size() > 0) {
				current = armature_best;
				/*
				while (true) {
					int j = (int)floor(rand() / (float)RAND_MAX * (tmp_pop.size() - 1));
					if (rand() / (float)RAND_MAX > tmp_fitness[j] / tmp_fitness_max - (tmp_fitness_min / tmp_fitness_max)) {
						current = tmp_pop[j];
						break;
					}
				}
				*/
			}
			armature_set_random_genetic(&current, s);
			armature_population.push_back(current);
			armature_fitness_f.push_back(armature_fitness(&current, onto_proj, s));
			if (fitness_max < armature_fitness_f[i]) fitness_max = armature_fitness_f[i];
			if (armature_fitness_f[i] < fitness_min) {
				fitness_min = armature_fitness_f[i];
				armature_best = current;
				fitness_best = fitness_min;
			}
		}

		vector<struct armature> armature_population_new;

		for (int g = 0; g < generations; g++) {
			armature_population_new.clear();
			armature_population_new.push_back(armature_best);
			for (int ra = 0; ra < population_size/16.0f; ra++) {
				struct armature n_ra = armature_best;
				armature_set_random_genetic(&n_ra, s);
				armature_population_new.push_back(n_ra);
			}
			while (armature_population_new.size() < population_size) {
				for (int i = 0; i < population_size; i++) {
					//int i = floor(rand() / (float)RAND_MAX * (population_size - 1));

					if (rand() / (float)RAND_MAX > armature_fitness_f[i] / fitness_max - (fitness_min / fitness_max)) {
						bool found_mate = false;
						struct armature* mate = nullptr;
						while (!found_mate) {
							int j = floor(rand() / (float)RAND_MAX * (population_size - 1));
							if (rand() / (float)RAND_MAX > armature_fitness_f[j] / fitness_max - (fitness_min / fitness_max)) {
								mate = &armature_population[j];
								found_mate = true;
								break;
							}
						}
						armature_population_new.push_back(armature_combine(&armature_population[i], mate, mutation_rate, s));
						if (armature_population_new.size() == population_size) break;
					}
				}
			}
			armature_population = armature_population_new;
			armature_fitness_f.clear();

			fitness_max = 0.0f;
			fitness_min = FLT_MAX;

			for (int i = 0; i < population_size; i++) {
				armature_fitness_f.push_back(armature_fitness(&armature_population[i], onto_proj, s));
				if (fitness_max < armature_fitness_f[i]) fitness_max = armature_fitness_f[i];
				if (armature_fitness_f[i] < fitness_min) {
					fitness_min = armature_fitness_f[i];
					fitness_min_idx = i;
					if (fitness_min < fitness_best) {
						armature_best = armature_population[i];
						fitness_best = fitness_min;
					}
				}
			}
			printf("stage: %i, fitness_max: %f, fitness_min: %f\n", s, fitness_max, fitness_min);
		}
		armature_print(&armature_best, true, s);
		printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n");
		armature_print(onto_proj, false, s);
	}
	printf("scale: %f\n", armature_best.scale);
	for (int i = 0; i < armature_bones.size(); i++) {
		struct bone *op_c = armature_bone_get_from_name(onto_proj, armature_bones[i]);
		struct bone* b_c_r = armature_bone_get_from_name(&armature_best, armature_bones[i]);
		struct bone b_c = armature_bone_get_abs_pos(&armature_best, armature_bones[i]);
		printf("%s: tail x: %f %f, y: %f %f, z: %f %f, head x: %f %f, y: %f %f, z: %f %f, bc %f, %f\n", armature_bones[i].c_str(), op_c->tail.x, b_c.tail.x, op_c->tail.y, b_c.tail.y, op_c->tail.z, b_c.tail.z, op_c->head.x, b_c.head.x, op_c->head.y, b_c.head.y, op_c->head.z, b_c.head.z, b_c_r->ball_coords.x, b_c_r->ball_coords.y);
	}
	
	armature_print(&armature_best, true, -1);
	printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n"); printf("\n");
	armature_print(onto_proj, false, -1);

	stringstream ss;
	ss << "output/" << w_idx << ".json";

	ofstream myfile;
	myfile.open(ss.str().c_str());
	myfile << "{\n";
	
	for (int i = 0; i < armature_bones.size(); i++) {
		struct bone b_c = armature_bone_get_abs_pos(&armature_best, armature_bones[i]);
		//printf("loc[%i] = [ %f, %f, %f ]\n", i * 2, b_c.tail.x, b_c.tail.y, b_c.tail.z);
		if (i > 0) myfile << ",";
		myfile << "\t\"" << armature_bones[i] << "\": {\n";
		myfile << "\t\t\"head\": [ " << b_c.head.x << ", " << b_c.head.y << ", " << b_c.head.z << " ],\n";
		myfile << "\t\t\"tail\": [ " << b_c.tail.x << ", " << b_c.tail.y << ", " << b_c.tail.z << " ]";
		myfile << "\t}\n ";
	}
	myfile << "}";
	myfile.close();
}

vector<struct armature_transformation> armature_normalise(struct armature* a, struct armature* base_norm) {
	vector<struct armature_transformation> result;
	//match spine
	struct vector3f spine_translation_param = { base_norm->spine.tail.x - a->spine.tail.x, base_norm->spine.tail.y - a->spine.tail.y, base_norm->spine.tail.z - a->spine.tail.z };
	struct armature_transformation spine_translation = { ATT_TRANSLATE_ALL, spine_translation_param };
	result.push_back(spine_translation);
	armature_translate_all_bones(a, spine_translation_param);
}

void armature_translate_all_bones(struct armature* a, struct vector3f translation) {
	for (int i = 0; i < armature_bones.size(); i++) {
		struct bone* current_bone = armature_bone_get_from_name(a, armature_bones[i]);
		current_bone->tail.x += translation.x;
		current_bone->tail.y += translation.y;
		current_bone->tail.z += translation.z;
		current_bone->head.x += translation.x;
		current_bone->head.y += translation.y;
		current_bone->head.z += translation.z;
	}
}

void armature_print(struct armature *a, bool abs, int stage) {
	int rows = 25;
	int cols = 50;

	int min_row_val = INT_MAX;
	int max_row_val = INT_MIN;

	int min_col_val = INT_MAX;
	int max_col_val = INT_MIN;

	char canvas[25*50];
	
	for (int i = 0; i < armature_bones.size(); i++) {
		if (stage == -1 || armature_gen_sets_bone_has_enabled_param(armature_bones[i], stage)) {
			struct bone* cur = nullptr;
			if (abs) {
				struct bone cur_b = armature_bone_get_abs_pos(a, armature_bones[i]);
				cur = &cur_b;
			} else {
				cur = armature_bone_get_from_name(a, armature_bones[i]);
			}
			if (floor(cur->tail.x) < min_col_val) min_col_val = floor(cur->tail.x);
			if (floor(cur->tail.x) > max_col_val) max_col_val = floor(cur->tail.x);
			if (floor(cur->tail.z) < min_row_val) min_row_val = floor(cur->tail.z);
			if (floor(cur->tail.z) > max_row_val) max_row_val = floor(cur->tail.z);
			if (floor(cur->head.x) < min_col_val) min_col_val = floor(cur->head.x);
			if (floor(cur->head.x) > max_col_val) max_col_val = floor(cur->head.x);
			if (floor(cur->head.z) < min_row_val) min_row_val = floor(cur->head.z);
			if (floor(cur->head.z) > max_row_val) max_row_val = floor(cur->head.z);
		}
	}
	
	for (int i = 0; i < rows*cols; i++) {
		canvas[i] = ' ';
	}

	for (int i = 0; i < armature_bones.size(); i++) {
		if (stage == -1 || armature_gen_sets_bone_has_enabled_param(armature_bones[i], stage)) {
			struct bone* cur = nullptr;
			struct bone cur_b = armature_bone_get_abs_pos(a, armature_bones[i]);
			if (abs) {
				cur = &cur_b;
			} else {
				cur = armature_bone_get_from_name(a, armature_bones[i]);
			}
			int col = (int)floor((floor(cur->tail.x) - min_col_val) / (float)(max_col_val - min_col_val) * (cols - 1));
			if (col < 0) col = 0;
			if (col > cols - 1) col = cols - 1;
			int row = (int)floor((floor(cur->tail.z) - min_row_val) / (float)(max_row_val - min_row_val) * (rows - 1));

			canvas[row * cols + col] = i + '0';

			col = (int)floor((floor(cur->head.x) - min_col_val) / (float)(max_col_val - min_col_val) * (cols - 1));
			if (col < 0) col = 0;
			if (col > cols - 1) col = cols - 1;
			row = (int)floor((floor(cur->head.z) - min_row_val) / (float)(max_row_val - min_row_val) * (rows - 1));

			canvas[row * cols + col] = i + '0';
		}
		
	}

	for (int i = 0; i < cols*rows; i++) {
		if (i % cols == 0) printf("\n");
		//printf("%c", canvas[cols * rows - i - 1]);
		printf("%c", canvas[i]);
		//if (i % cols == cols-1) printf("\n");
	}
		
}

float armature_bone_length(struct bone* b) {
	struct vector3f d = { b->head.x-b->tail.x, b->head.y-b->tail.y, b->head.z-b->tail.z};
	return sqrtf(d.x * d.x + d.y * d.y + d.z * d.z);
}

struct bone* armature_bone_get_from_name(struct armature* a, string bone_name) {
	struct bone* result = nullptr;
	if (bone_name.compare("arm_lo_l") == 0) {
		result = &a->arm_lo_l;
	} else if (bone_name.compare("arm_lo_r") == 0) {
		result = &a->arm_lo_r;
	} else if (bone_name.compare("arm_up_l") == 0) {
		result = &a->arm_up_l;
	} else if (bone_name.compare("arm_up_r") == 0) {
		result = &a->arm_up_r;
	} else if (bone_name.compare("foot_l") == 0) {
		result = &a->foot_l;
	} else if (bone_name.compare("foot_r") == 0) {
		result = &a->foot_r;
	} else if (bone_name.compare("head") == 0) {
		result = &a->head;
	} else if (bone_name.compare("leg_lo_l") == 0) {
		result = &a->leg_lo_l;
	} else if (bone_name.compare("leg_lo_r") == 0) {
		result = &a->leg_lo_r;
	} else if (bone_name.compare("leg_up_l") == 0) {
		result = &a->leg_up_l;
	} else if (bone_name.compare("leg_up_r") == 0) {
		result = &a->leg_up_r;
	} else if (bone_name.compare("shoulder_l") == 0) {
		result = &a->shoulder_l;
	} else if (bone_name.compare("shoulder_r") == 0) {
		result = &a->shoulder_r;
	} else if (bone_name.compare("spine") == 0) {
		result = &a->spine;
	} else if (bone_name.compare("spine_l") == 0) {
		result = &a->spine_l;
	} else if (bone_name.compare("spine_r") == 0) {
		result = &a->spine_r;
	} else if (bone_name.compare("thorax") == 0) {
		result = &a->thorax;
	}
	return result;
}

//FIXME:
struct bone armature_bone_get_abs_pos(struct armature* a, string bone_name) {
	struct bone result;
	result.tail = { a->spine.tail.x, a->spine.tail.y, a->spine.tail.z };
	result.head = { 0.0f, 0.0f, 0.0f };
	struct vector3f rotation = a->spine.ball_coords;
	struct bone* current_bone;
	struct bone tmp;
	struct bone tmp_last;
	tmp_last.ball_coords = { a->spine.ball_coords.x, a->spine.ball_coords.y+a->spine.ball_coords.z, a->spine.ball_coords.z };
	for (int i = 0; i < armature_bone_dependencies[bone_name].size(); i++) {
		current_bone = armature_bone_get_from_name(a, armature_bone_dependencies[bone_name][i]);
		
		tmp.tail.x = 0.0f;
		tmp.tail.y = 0.0f;
		tmp.tail.z = 0.0f;
		tmp.head.x = current_bone->head.x - current_bone->tail.x;
		tmp.head.y = current_bone->head.y - current_bone->tail.y;
		tmp.head.z = current_bone->head.z - current_bone->tail.z;

		tmp.ball_coords = { acosf(tmp.head.z / armature_bone_length(current_bone)), atan2f(tmp.head.y, tmp.head.x) };

		struct vector2f ball_coords_d = { tmp.ball_coords.x - tmp_last.ball_coords.x, tmp.ball_coords.y - tmp_last.ball_coords.y };
		
		if (armature_bone_dependencies[bone_name][i].compare("spine") != 0) {
			rotation.x += ball_coords_d.x + current_bone->ball_coords.x;
			rotation.y += ball_coords_d.y + current_bone->ball_coords.y + current_bone->ball_coords.z;
		}

		result.tail.x += armature_bone_length(current_bone) * a->scale * sinf(rotation.x) * cosf(rotation.y);
		result.tail.y += armature_bone_length(current_bone) * a->scale * sinf(rotation.x) * sinf(rotation.y);
		result.tail.z += armature_bone_length(current_bone) * a->scale * cosf(rotation.x);

		tmp_last.ball_coords = tmp.ball_coords;
	}

	//bone head
	current_bone = armature_bone_get_from_name(a, bone_name);

	tmp.tail.x = 0.0f;
	tmp.tail.y = 0.0f;
	tmp.tail.z = 0.0f;
	tmp.head.x = current_bone->head.x - current_bone->tail.x;
	tmp.head.y = current_bone->head.y - current_bone->tail.y;
	tmp.head.z = current_bone->head.z - current_bone->tail.z;

	tmp.ball_coords = { acosf(tmp.head.z / armature_bone_length(current_bone)), atan2f(tmp.head.y, tmp.head.x) };

	struct vector2f ball_coords_d = { tmp.ball_coords.x - tmp_last.ball_coords.x, tmp.ball_coords.y - tmp_last.ball_coords.y };

	if (bone_name.compare("spine") != 0) {
		rotation.x += ball_coords_d.x + current_bone->ball_coords.x;
		rotation.y += ball_coords_d.y + current_bone->ball_coords.y + current_bone->ball_coords.z;
	}

	result.head.x = result.tail.x + armature_bone_length(current_bone) * a->scale * sinf(rotation.x) * cosf(rotation.y);
	result.head.y = result.tail.y + armature_bone_length(current_bone) * a->scale * sinf(rotation.x) * sinf(rotation.y);
	result.head.z = result.tail.z + armature_bone_length(current_bone) * a->scale * cosf(rotation.x);
	return result;
}
