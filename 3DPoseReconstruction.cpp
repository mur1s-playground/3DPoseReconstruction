#include "3DPoseReconstruction.hpp"

#include <iostream>

#include "Util.hpp"
#include "Keypoint.hpp"
#include "Armature.hpp"

#include <random>

std::random_device r;
std::seed_seq seed{ r(), r(), r(), r(), r(), r(), r(), r() };
std::mt19937 eng(seed);
std::uniform_int_distribution<> dist{ 0, RAND_MAX };

#define rand() myrand()

int myrand() {
    return dist(eng);
}


int main() {
    
    vector<string> filenames = get_all_files_names_within_folder("C:\\Users\\mur1_\\Downloads\\shuffle.json\\", "*", "json");

    vector<vector<struct keypoint>> keypoints;
    for (int i = 0; i < filenames.size(); i++) {
        //printf("%s\n", filenames[i].c_str());
        keypoints.push_back(get_cfg_key_value_pairs("C:\\Users\\mur1_\\Downloads\\shuffle.json\\", filenames[i]));
        for (int j = 0; j < keypoints[i].size(); j++) {
            if (keypoints[i][j].x == 0 || keypoints[i][j].y == 0 || keypoints[i][j].confidence == 0) {
                keypoints[i].clear();
                break;
            }
           // printf("%f %f %f\n", keypoints[i][j].x, keypoints[i][j].y, keypoints[i][j].confidence);
        }
    }
    
    float head_len = 0;
    float spine_len = 0;
    float thorax_len = 0;
    float shoulder_radius = 0;
    float arm_up_len = 0;
    float arm_lo_len = 0;
    float spine_radius = 0;
    float leg_up_len = 0;
    float leg_lo_len = 0;
    float foot_len = 0;

    int valid_i = 0;

    for (int i = 450; i < 1000; i++) {
        if (keypoints[i].size() > 0) {
            if (valid_i == 0) valid_i = i;
            struct vector2f head = { keypoints[i][KEYPOINT_HEAD].x   , keypoints[i][KEYPOINT_HEAD].y };
            struct vector2f shoulder = { keypoints[i][KEYPOINT_THORAX].x   , keypoints[i][KEYPOINT_THORAX].y };

            struct vector2f hs_d = { head.x - shoulder.x, head.y - shoulder.y };
            float hs_norm = sqrtf(hs_d.x * hs_d.x + hs_d.y * hs_d.y);
            if (hs_norm > head_len) head_len = hs_norm;

            
            struct vector2f shoulder_l = { keypoints[i][KEYPOINT_LEFT_SHOULDER].x , keypoints[i][KEYPOINT_LEFT_SHOULDER].y };
            struct vector2f shoulder_r = { keypoints[i][KEYPOINT_RIGHT_SHOULDER].x , keypoints[i][KEYPOINT_RIGHT_SHOULDER].y };

            struct vector2f shoulder_l_d = { shoulder_l.x - shoulder.x, shoulder_l.y - shoulder.y };
            struct vector2f shoulder_r_d = { shoulder_r.x - shoulder.x, shoulder_r.y - shoulder.y };

            float shoulder_l_norm = sqrtf(shoulder_l_d.x * shoulder_l_d.x + shoulder_l_d.y * shoulder_l_d.y);
            float shoulder_r_norm = sqrtf(shoulder_r_d.x * shoulder_r_d.x + shoulder_r_d.y * shoulder_r_d.y);
            if (shoulder_l_norm > shoulder_radius) shoulder_radius = shoulder_l_norm;
            if (shoulder_r_norm > shoulder_radius) shoulder_radius = shoulder_r_norm;

            struct vector2f spine   = { keypoints[i][KEYPOINT_SPINE].x   , keypoints[i][KEYPOINT_SPINE].y };
            struct vector2f spine_l = { keypoints[i][KEYPOINT_LEFT_SPINE].x   , keypoints[i][KEYPOINT_LEFT_SPINE].y };
            struct vector2f spine_r = { keypoints[i][KEYPOINT_RIGHT_SPINE].x  , keypoints[i][KEYPOINT_RIGHT_SPINE].y };
            
            struct vector2f spine_l_d = { spine_l.x - spine.x, spine_l.y - spine.y };
            struct vector2f spine_r_d = { spine_r.x - spine.x, spine_r.y - spine.y };

            float spine_l_norm = sqrtf(spine_l_d.x * spine_l_d.x + spine_l_d.y * spine_l_d.y);
            float spine_r_norm = sqrtf(spine_r_d.x * spine_r_d.x + spine_r_d.y * spine_r_d.y);
            if (spine_l_norm > spine_radius) spine_radius = spine_l_norm;
            if (spine_r_norm > spine_radius) spine_radius = spine_r_norm;

            struct vector2f ss_d = { spine.x-shoulder.x, spine.y-shoulder.y};
            float ss_norm = sqrtf(ss_d.x * ss_d.x + ss_d.y * ss_d.y);

            if (ss_norm * 0.5 > thorax_len) {
                thorax_len = ss_norm * 0.5;
                spine_len = ss_norm * 0.5;
            }

            struct vector2f arm_up_l = { keypoints[i][KEYPOINT_LEFT_ARM_UP].x   , keypoints[i][KEYPOINT_LEFT_ARM_UP].y };
            struct vector2f arm_up_r = { keypoints[i][KEYPOINT_RIGHT_ARM_UP].x  , keypoints[i][KEYPOINT_RIGHT_ARM_UP].y };

            struct vector2f arm_up_l_d = { arm_up_l.x - shoulder_l.x, arm_up_l.y - shoulder_l.y };
            struct vector2f arm_up_r_d = { arm_up_r.x - shoulder_r.x, arm_up_r.y - shoulder_r.y };

            float arm_up_l_norm = sqrtf(arm_up_l_d.x * arm_up_l_d.x + arm_up_l_d.y * arm_up_l_d.y);
            float arm_up_r_norm = sqrtf(arm_up_r_d.x * arm_up_r_d.x + arm_up_r_d.y * arm_up_r_d.y);
            if (arm_up_l_norm > arm_up_len) arm_up_len = arm_up_l_norm;
            if (arm_up_r_norm > arm_up_len) arm_up_len = arm_up_r_norm;

            struct vector2f arm_lo_l = { keypoints[i][KEYPOINT_LEFT_ARM_LO].x   , keypoints[i][KEYPOINT_LEFT_ARM_LO].y };
            struct vector2f arm_lo_r = { keypoints[i][KEYPOINT_RIGHT_ARM_LO].x  , keypoints[i][KEYPOINT_RIGHT_ARM_LO].y };

            struct vector2f arm_lo_l_d = { arm_up_l.x - arm_lo_l.x, arm_up_l.y - arm_lo_l.y };
            struct vector2f arm_lo_r_d = { arm_up_r.x - arm_lo_r.x, arm_up_r.y - arm_lo_r.y };

            float arm_lo_l_norm = sqrtf(arm_lo_l_d.x * arm_lo_l_d.x + arm_lo_l_d.y * arm_lo_l_d.y);
            float arm_lo_r_norm = sqrtf(arm_lo_r_d.x * arm_lo_r_d.x + arm_lo_r_d.y * arm_lo_r_d.y);
            if (arm_lo_l_norm > arm_lo_len) arm_lo_len = arm_lo_l_norm;
            if (arm_lo_r_norm > arm_lo_len) arm_lo_len = arm_lo_r_norm;

            struct vector2f leg_up_l = { keypoints[i][KEYPOINT_LEFT_LEG_UP].x   , keypoints[i][KEYPOINT_LEFT_LEG_UP].y };
            struct vector2f leg_up_r = { keypoints[i][KEYPOINT_RIGHT_LEG_UP].x  , keypoints[i][KEYPOINT_RIGHT_LEG_UP].y };

            struct vector2f leg_up_l_d = { leg_up_l.x - spine_l.x, leg_up_l.y - spine_l.y };
            struct vector2f leg_up_r_d = { leg_up_r.x - spine_r.x, leg_up_r.y - spine_r.y };

            float leg_up_l_norm = sqrtf(leg_up_l_d.x * leg_up_l_d.x + leg_up_l_d.y * leg_up_l_d.y);
            float leg_up_r_norm = sqrtf(leg_up_r_d.x * leg_up_r_d.x + leg_up_r_d.y * leg_up_r_d.y);
            if (leg_up_l_norm > leg_up_len) leg_up_len = leg_up_l_norm;
            if (leg_up_r_norm > leg_up_len) leg_up_len = leg_up_r_norm;

            struct vector2f leg_lo_l = { (keypoints[i][KEYPOINT_LEFT_LEG_LO].x + keypoints[i][KEYPOINT_LEFT_LEG_LO_F].x) * 0.5   , (keypoints[i][KEYPOINT_LEFT_LEG_LO].y + keypoints[i][KEYPOINT_LEFT_LEG_LO_F].y) * 0.5 };
            struct vector2f leg_lo_r = { (keypoints[i][KEYPOINT_RIGHT_LEG_LO].x + keypoints[i][KEYPOINT_RIGHT_LEG_LO_F].x) * 0.5   , (keypoints[i][KEYPOINT_RIGHT_LEG_LO].y + keypoints[i][KEYPOINT_RIGHT_LEG_LO_F].y) * 0.5 };

            struct vector2f leg_lo_l_d = { leg_up_l.x - leg_lo_l.x, leg_up_l.y - leg_lo_l.y };
            struct vector2f leg_lo_r_d = { leg_up_r.x - leg_lo_r.x, leg_up_r.y - leg_lo_r.y };

            float leg_lo_l_norm = sqrtf(leg_lo_l_d.x * leg_lo_l_d.x + leg_lo_l_d.y * leg_lo_l_d.y);
            float leg_lo_r_norm = sqrtf(leg_lo_r_d.x * leg_lo_r_d.x + leg_lo_r_d.y * leg_lo_r_d.y);
            if (leg_lo_l_norm > leg_lo_len) leg_lo_len = leg_lo_l_norm;
            if (leg_lo_r_norm > leg_lo_len) leg_lo_len = leg_lo_r_norm;

            struct vector2f foot_l = { (keypoints[i][KEYPOINT_LEFT_FOOT_1].x + keypoints[i][KEYPOINT_LEFT_FOOT_2].x) * 0.5   , (keypoints[i][KEYPOINT_LEFT_FOOT_1].y + keypoints[i][KEYPOINT_LEFT_FOOT_2].y) * 0.5 };
            struct vector2f foot_r = { (keypoints[i][KEYPOINT_RIGHT_FOOT_1].x + keypoints[i][KEYPOINT_RIGHT_FOOT_2].x) * 0.5   , (keypoints[i][KEYPOINT_RIGHT_FOOT_1].y + keypoints[i][KEYPOINT_RIGHT_FOOT_2].y) * 0.5 };

            struct vector2f foot_l_d = { foot_l.x - leg_lo_l.x, foot_l.y - leg_lo_l.y };
            struct vector2f foot_r_d = { foot_r.x - leg_lo_r.x, foot_r.y - leg_lo_r.y };

            float foot_l_norm = sqrtf(foot_l_d.x * foot_l_d.x + foot_l_d.y * foot_l_d.y);
            float foot_r_norm = sqrtf(foot_r_d.x * foot_r_d.x + foot_r_d.y * foot_r_d.y);
            if (foot_l_norm > foot_len) foot_len = foot_l_norm;
            if (foot_r_norm > foot_len) foot_len = foot_r_norm;
        }
    }
    printf("head_len: %f\n", head_len);
    printf("thorax_len: %f\n", thorax_len);
    printf("spine_len: %f\n", spine_len);
    printf("shoulder_radius: %f\n", shoulder_radius);
    printf("arm_up_len: %f\n", arm_up_len);
    printf("arm_lo_len: %f\n", arm_lo_len);
    printf("spine_radius: %f\n", spine_radius);
    printf("leg_up_len: %f\n", leg_up_len);
    printf("leg_lo_len: %f\n", leg_lo_len);
    printf("foot_len: %f\n", foot_len);

    struct armature a;
    armature_base_init(&a, head_len, spine_len, thorax_len, shoulder_radius, arm_up_len, arm_lo_len, spine_radius, leg_up_len, leg_lo_len, foot_len);

    for (int i = 0; i < armature_bones.size(); i++) {
        struct bone* b_c = armature_bone_get_from_name(&a, armature_bones[i]);
        printf("%s: %f %f %f, %f, %f, %f %f %f\n", armature_bones[i].c_str(), b_c->tail.x, b_c->tail.y, b_c->tail.z, b_c->ball_coords.x, b_c->ball_coords.y, b_c->head.x, b_c->head.y, b_c->head.z);
    }
    //armature_print(&a, 0.1);

    printf("proj_450\n");
    struct armature a_proj_450;
    printf("kp_s %i\n", keypoints[valid_i].size());
    armature_proj_from_keypoints(&a_proj_450, keypoints[valid_i]);

    for (int i = 0; i < armature_bones.size(); i++) {
        struct bone* b_c = armature_bone_get_from_name(&a_proj_450, armature_bones[i]);
        printf("%s: %f %f %f, %f, %f, %f %f %f\n", armature_bones[i].c_str(), b_c->tail.x, b_c->tail.y, b_c->tail.z, b_c->ball_coords.x, b_c->ball_coords.y, b_c->head.x, b_c->head.y, b_c->head.z);
    }
    //armature_print(&a_proj_450, 0.1);
    printf("proj_450 done\n");

    printf("starting fit\n");

    armature_fit(&a, &a_proj_450);
}