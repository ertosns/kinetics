#pragma once
#include <iostream>
#include <string>
#include <filesystem>
#include <exception>
#include <fstream>
#include <sstream>
#include <regex>
#include <map>

namespace fs = std::filesystem;
using namespace std;

class NotConfig : public exception {
    string what() {
        return string("not a config file");
    }
};

class Config {
    const fs::path f;
    std::map<string,string> dict;
public:
    Config(fs::path filepath) : f(filepath) {
        if (!fs::exists(f)) {
            std::cout << "path donesn't exist filepath: " << f << std::endl;
            throw new NotConfig();
        }
        read_config();
    }
    string operator[](string key) {
        return dict[key];
    }
    bool has(string key) {
        return dict.find(key)!= dict.end();
    }
private:
    void read_config() {
        string EQ_DEL("=");
        std::ifstream ifs(f);
        std::stringstream buff_ss; //configfile buffer
        buff_ss << ifs.rdbuf();
        string buff = buff_ss.str();
        //ignore comments
        smatch m;
        //regex e("[^#][[:w:]]*=[[:d:]]*");
        regex e("([[:w:]_]*)=([_[:w:].[:d:]]*)");
        sregex_iterator pos(buff.cbegin(), buff.cend(), e);
        sregex_iterator end;
        for (; pos!=end; pos++) {
            //pos->str(0);
            string key = pos->str(1);
            string val = pos->str(2);
            dict.insert(pair<string,string>(key, val));
        }
        /*
        bool match = regex_search(buff, m, e);
        for (int i=0; i < m.size(); i++) {
            string keyval = m[i].str();
            vector<string> kv = split(keyval, EQ_DEL);
            dict.insert(pair<string,string>(kv[0], kv[1]));
            std::cout << "key: " << kv[0] << ", val: " << kv[1] << std::endl;
        }
        */
        //read key-value pairs
        //separate them into two string, and push those pair to the map
    }
    vector<string> split (string s, string delimiter) {
        size_t pos_start = 0, pos_end, delim_len = delimiter.length();
        string token;
        vector<string> res;

        while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
            token = s.substr (pos_start, pos_end - pos_start);
            pos_start = pos_end + delim_len;
            res.push_back (token);
        }

        res.push_back (s.substr (pos_start));
        return res;
    }
};
