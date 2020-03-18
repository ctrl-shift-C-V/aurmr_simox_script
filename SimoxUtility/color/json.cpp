#include "json.h"


namespace simox
{

    void color::to_json(nlohmann::json& j, const Color& color)
    {
        j["r"] = color.r;
        j["g"] = color.g;
        j["b"] = color.b;
        j["a"] = color.a;
    }

    void color::from_json(const nlohmann::json& j, Color& color)
    {
        color.r = j.at("r");
        color.g = j.at("g");
        color.b = j.at("b");
        color.a = j.at("a");
    }

}


