#include <cpr/cpr.h>
#include <iostream>
#include <string>
#include "jasonParser.hpp"

class HttpRequest {
public:
    bool isOpen() {
        auto r = cpr::Get(cpr::Url{"http://www.httpbin.org/get"});

        std::cout << "parser by tianyi:  " << std::endl;

        JasonParser jason(r.text);

        std::cout << jason.getValue("origin") << std::endl;

		return true;
    }
};
