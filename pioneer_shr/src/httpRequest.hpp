#include <cpr/cpr.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include "jasonParser.hpp"

class HttpRequest {
public:
    bool isOpen() {
        auto r = cpr::Get(cpr::Url{"http://localhost:4567/getcontacts"});

        std::string webtext = r.text;

        std::string jasonListStr = webtext.substr(1, webtext.size() - 2);

        std::stringstream ss(jasonListStr);

        std::vector<std::string> jasonStrList;

        std::string jasonStr;

        int i = 1;
        while (getline(ss, jasonStr, ',')) {
            if (i % 2 != 0)
                jasonStrList.push_back(jasonStr);
            else
                jasonStrList[jasonStrList.size() - 1] += "," + jasonStr;

            i++;
        }


        std::vector<JasonParser> jasonList;

        for (auto str : jasonStrList) {
            jasonList.push_back(JasonParser(str));
        }

        std::cout << "parser by tianyi:  " << std::endl;

        // JasonParser jason(r.body);

        // std::cout << jason.getValue("origin") << std::endl;

        std::cout << jasonList[0].getValue("value") << std::endl;

        if (jasonList[0].getValue("value") == "open")
            return true;
        else
            return false;
    }
};
