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

        while (getline(ss, jasonStr, ',')) {
            jasonStrList.push_back(jasonStr);
        }

        std::cout << "parser by tianyi:  " << std::endl;

        // JasonParser jason(r.body);

        // std::cout << jason.getValue("origin") << std::endl;

        std::cout << jasonStrList[0] << std::endl;

        return true;
    }
};
