#include <cpr/cpr.h>
#include <iostream>

int main(int argc, char** argv) {
    auto r = cpr::Get(cpr::Url{"http://www.httpbin.org/get"});

    std::cout << r.url << std::endl;
    std::cout << r.status_code << std::endl;
    std::cout << r.header["content-type"] << std::endl;
    std::cout << r.text << std::endl;
}
