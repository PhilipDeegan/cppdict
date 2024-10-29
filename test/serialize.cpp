
#include "dict.hpp"
#include "dict/exp/serializer.hpp"


int main()
{
    using MyDict = cppdict::Dict<int, double, std::string, std::vector<int>>;

    std::string const file("outfile.dat");

    {
        MyDict md;
        md["test"]["super"] = 2;
        md["PI"]            = 3.14;
        md["Key"]           = std::string{"Value"};
        md["key2"]          = 2;
        md["vec"]           = std::vector<int>{1, 2, 3};

        cppdict::DictSerializer<int, double, std::string, std::vector<int>>{file}(md);
    }

    auto md = cppdict::DictDeSerializer<int, double, std::string, std::vector<int>>{file}();

    auto abort_if_not = [](auto const a, auto const b) {
        if (a != b)
            std::abort();
    };

    abort_if_not(md["test"]["super"].to<int>(), 2);
    abort_if_not(md["PI"].to<double>(), 3.14);
    abort_if_not(md["Key"].to<std::string>(), "Value");
    abort_if_not(md["key2"].to<int>(), 2);
    return md["vec"].to<std::vector<int>>() != std::vector<int>{1, 2, 3};
}
