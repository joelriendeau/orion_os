#include "settings.hpp"

class v1_0_0 : public settings::base
{
public:
    v1_0_0()
    {
        define_setting<std::string>("version", "1.0.0");
        define_setting<double>("rate", 1.12154);
        define_setting<u32>("unexisting", 13);
    }
};

class v1_0_1 : public settings::base
{
public:
    v1_0_1()
    {
        define_setting<std::string>("version", "1.0.1");
        define_setting<double>("rate", 1.12154, convert_rate_from_1_0_0);
        define_setting<double>("rate_sup", 2);
        define_setting<std::string>("music", "nick_drake");
        define_old_version<v1_0_0>();
    }

    static void convert_rate_from_1_0_0(settings::base::settings_map_type& old, settings::base::settings_map_type& current)
    {
        double old_rate;
        old.get("rate", old_rate);
        old_rate += 1;
        current.set("rate", old_rate);
    }
};

class v1_0_2 : public settings::base
{
public:
    v1_0_2()
    {
        define_setting<std::string>("version", "1.0.2");
        define_setting<double>("rate", 4., convert_rate_from_1_0_1);
        define_setting<std::string>("music", "nick_drake");
        define_old_version<v1_0_1>();
    }

    static void convert_rate_from_1_0_1(settings::base::settings_map_type& old, settings::base::settings_map_type& current)
    {
        double old_rate;
        old.get("rate", old_rate);
        double old_rate_sup;
        old.get("rate_sup", old_rate_sup);
        old_rate += old_rate_sup;
        current.set("rate", old_rate);
    }
};

int main(int argc, char* argv[])
{
    settings::text_file_loader parser;
    std::map<std::string, std::string> key_val;
    std::string text = "version =1.0.0\n\r\nrate=    1.223344";
    parser.load(text, key_val);

    v1_0_2 test;
    test.load(key_val);

    std::string version;
    test.get("version", version);
    double rate;
    test.get("rate", rate);

	return 0;
}

