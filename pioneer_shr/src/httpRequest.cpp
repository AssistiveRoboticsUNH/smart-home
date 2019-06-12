#include <cpr/cpr.h>
#include <iostream>
#include <string>
#include "jasonParser.hpp"

int main(int argc, char** argv) {
    auto r = cpr::Get(cpr::Url{"http://www.httpbin.org/get"});

    std::cout << "parser by tianyi:  " << std::endl;

	JasonParser jason(r.text);

    std::cout << jason.getValue("origin") << std::endl;
}
