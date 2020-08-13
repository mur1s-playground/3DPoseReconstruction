#pragma once

#undef UNICODE
#undef _UNICODE

#include <windows.h>
#include <fstream>
#include <vector>
#include <string>

using namespace std;

vector<string> get_all_files_names_within_folder(string folder, string wildcard, string extension);
vector<struct keypoint> get_cfg_key_value_pairs(string folder, string filename);