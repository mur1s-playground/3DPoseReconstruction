#include "Util.hpp"

#include "Keypoint.hpp"

vector<string> get_all_files_names_within_folder(string folder, string wildcard, string extension) {
    vector<string> names;
    string search_path = folder + "/" + wildcard + "." + extension;
    WIN32_FIND_DATA fd;
    HANDLE hFind = ::FindFirstFile(search_path.c_str(), &fd);
    if (hFind != INVALID_HANDLE_VALUE) {
        do {
            if (!(fd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) {
                names.push_back(fd.cFileName);
            }
        } while (::FindNextFile(hFind, &fd));
        ::FindClose(hFind);
    }
    return names;
}

vector<struct keypoint> get_cfg_key_value_pairs(string folder, string filename) {
    string filepath = folder + "/" + filename;
    vector<struct keypoint> result;

    ifstream file(filepath);
    string filecontent;
    if (file.is_open()) {
        while (std::getline(file, filecontent)) {
            if (filecontent.size() > 0) {
                size_t last_pos = 0;
                string start_s = "\"pose_keypoints_2d\":[";
                size_t pos = filecontent.find(start_s);

                size_t end = filecontent.find("]", pos);
                if (pos != string::npos) {
                    pos += start_s.length();                

                    int mod = 0;
                    struct keypoint kp;

                    while (pos < end) {
                        size_t komma = filecontent.find(",", pos);
                        string value = "";
                        if (komma > end) {
                            value = filecontent.substr(pos, end - pos);
                            pos = end + 1;
                        } else {
                            value = filecontent.substr(pos, komma - pos);
                            pos = komma + 1;
                        }
                        if (mod == 0) {
                            kp.x = stof(value);
                        } else if (mod == 1) {
                            kp.y = stof(value);
                        } else if (mod == 2) {
                            kp.confidence = stof(value);
                            result.push_back(kp);
                        }
                        mod = (mod + 1) % 3;
                        //printf("%i %i %s\n", pos, komma, value.c_str());
                    }
                    
                    //printf("cfg key: %s, value: %s\n", name.c_str(), value.c_str());
                }
            }
        }
    }
    return result;
}