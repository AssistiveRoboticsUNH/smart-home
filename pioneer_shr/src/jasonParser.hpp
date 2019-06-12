// rapidjson/example/simpledom/simpledom.cpp`
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include <iostream>
#include <string>
using namespace rapidjson;
//int main() {
    //// 1. Parse a JSON string into DOM.
    //const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    //Document d;
    //d.Parse(json);
    //// 2. Modify it by DOM.
    //Value& s = d["stars"];
    //s.SetInt(s.GetInt() + 1);
    //// 3. Stringify the DOM
    //StringBuffer buffer;
    //Writer<StringBuffer> writer(buffer);
    //d.Accept(writer);
    //// Output {"project":"rapidjson","stars":11}
    //std::cout << buffer.GetString() << std::endl;
    //return 0;
/*}*/

class JasonParser {
public:
    JasonParser(const std::string& jasonStr) { jasonDoc.Parse(jasonStr.c_str()); }

	std::string getValue(const char* key) const {
        const Value& v = jasonDoc[key];
        return v.GetString();
    }

private:
    Document jasonDoc;
};

