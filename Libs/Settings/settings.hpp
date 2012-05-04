#pragma once

#include "types.hpp"
#include "boost/any.hpp"
#ifndef __CROSSWORKS_ARM
    #ifdef WINCE
        #define BOOST_NO_STD_LOCALE
        #define BOOST_LEXICAL_CAST_ASSUME_C_LOCALE
    #endif
    #include "boost/lexical_cast.hpp"
#else
    #include "stdlib.h"
#endif
#include "boost/tokenizer.hpp"
#include <map>

#ifdef __CROSSWORKS_ARM
namespace boost {
    // simple reimplementation of lexical_cast using a preallocated buffer. CrossWorks does not support iostreams, and lexical_cast has
    // no fallback for that case
    template <typename Dest, typename Source>
    inline Dest lexical_cast(const Source& arg)
    {
        char buf[24]; // a u64 number would take 20 digits max. this seems safe.
        sprintf(buf, "%d", arg);
        return std::string(buf);
    }
    template <>
    inline std::string lexical_cast<std::string, std::string>(const std::string& arg)
    {
        return arg;
    }
    template <>
    inline std::string lexical_cast<std::string, float>(const float& arg)
    {
        char buf[24];
        sprintf(buf, "%.7f", arg);
        return std::string(buf);
    }
    template <>
    inline std::string lexical_cast<std::string, double>(const double& arg)
    {
        char buf[24];
        sprintf(buf, "%.16f", arg);
        return std::string(buf);
    }
    template <>
    inline u8 lexical_cast<u8, std::string>(const std::string& arg)
    {
        u8 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline u16 lexical_cast<u16, std::string>(const std::string& arg)
    {
        u16 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline u32 lexical_cast<u32, std::string>(const std::string& arg)
    {
        u32 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline s8 lexical_cast<s8, std::string>(const std::string& arg)
    {
        s8 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline s16 lexical_cast<s16, std::string>(const std::string& arg)
    {
        s16 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline s32 lexical_cast<s32, std::string>(const std::string& arg)
    {
        s32 r = atoi(arg.c_str());
        return r;
    }
    template <>
    inline float lexical_cast<float, std::string>(const std::string& arg)
    {
        float r = strtod(arg.c_str(), 0);
        return r;
    }
    template <>
    inline double lexical_cast<double, std::string>(const std::string& arg)
    {
        double r = strtod(arg.c_str(), 0);
        return r;
    }
}
#endif

namespace settings {

class base
{
public:
    base() : old_version(0) {}

    ~base() {if (old_version) delete old_version;}

    template <typename T>
    bool get(const std::string& name, T& output)
    {
        return settings_map.get(name, output);
    }

    template <typename T>
    void set(const std::string& name, T& new_value)
    {
        settings_map.set(name, new_value);
    }

    void load(std::map<std::string, std::string>& key_value)
    {
        settings_map_type om;
        convert(key_value, om); // convert from the old version to this version
        settings_map.swap(om);
    }

    struct settings_map_type;
    struct settings_node
    {
        boost::any value;
        boost::any default_value;
        void (*converter)(settings_map_type&, settings_map_type&);
        boost::any  (*from_string)(const std::string&);
        std::string (*to_string)  (const boost::any&);
    };

    struct settings_map_type : public std::map<std::string, settings_node>
    {
        template <typename T>
        bool get(const std::string& name, T& output)
        {
            iterator it = find(name);
            if (end() == it)
                return false;

            output = boost::any_cast<T>(it->second.value);
            return true;
        }

        template <typename T>
        void set(const std::string& name, T& new_value)
        {
            operator[](name).value = new_value;
        }
    };

    settings_map_type& get_settings_map()
    {
        return settings_map;
    }

protected:
    template <typename T>
    static boost::any default_from_string(const std::string& value)
    {
        return boost::lexical_cast<T>(value);
    }

    template <typename T>
    static std::string default_to_string(const boost::any& value)
    {
        return boost::lexical_cast<std::string>(boost::any_cast<T>(value));
    }

    template <typename T>
    void define_setting(const std::string& name, T default_value, void (*conversion)(settings_map_type&, settings_map_type&) = 0,
                        boost::any  (*from_string)(const std::string&) = default_from_string<T>,
                        std::string (*to_string)(const boost::any&) = default_to_string<T>)
    {
        settings_node node;
        node.value = node.default_value = default_value;
        node.converter = conversion;
        node.from_string = from_string;
        node.to_string = to_string;
        settings_map[name] = node;
    }

    template <typename T>
    void define_old_version()
    {
        old_version = new T;
    }

    void convert(std::map<std::string, std::string>& key_value, settings_map_type& output_map)
    {
        bool older, same = false;

        std::map<std::string, std::string>::iterator it = key_value.find("version");
        if (key_value.end() != it) // the data loaded had a version stamp
        {
            std::string current_version_string;
            get("version", current_version_string);

            is_older(it->second, current_version_string, same, older);

            if (older && old_version) // if the data looks older than our version, and an older version is defined, ask it to convert first
            {
                old_version->convert(key_value, output_map);
                upgrade(output_map);
            }
        }

        // for each element of this version, check if a value was supplied by the upgrade process, and use it in place of the default.
        // if there was no upgrade, the try to use the key_value pairs directly.
        std::map<std::string, std::string>::iterator kv_it;
        settings_map_type::iterator om_it;
        for (settings_map_type::iterator it = settings_map.begin();
            it != settings_map.end();
            ++it)
        {
            if (same)
            {
                kv_it = key_value.find(it->first);
                if (key_value.end() != kv_it)
                    it->second.value = it->second.from_string(kv_it->second);
                else
                    it->second.value = it->second.default_value;
            }
            else
            {
                om_it = output_map.find(it->first);
                if (output_map.end() != om_it)
                    it->second.value = om_it->second.value;
                else
                    it->second.value = it->second.default_value;
            }
        }
        output_map.swap(settings_map);
    }

    void upgrade(settings_map_type& output_map)
    {
        if (!old_version)
            return;

        std::string version;
        if (!get("version", version))
            return;

        std::string previous_version;
        old_version->get("version", previous_version);
        if (previous_version == version)
            return;

        settings_map_type target_map;
        for (settings_map_type::iterator it = settings_map.begin();
            it != settings_map.end();
            ++it)
        {
            if (it->first == "version")
                continue;
            else if (it->second.converter)
                it->second.converter(output_map, target_map);
        }
        output_map.swap(target_map);

        output_map.set("version", version); // upgrade the version stamp
    }

private:
    void is_older(std::string& a, std::string& b, bool& same, bool& a_older_than_b) // is a older than b?
    {
        // versions should used a dotted notation (a.b.c.d, e.g. 3.4.1)
        // but we also support these characters for more complex version ids : '_', '-', '.'
        // they are all treated the same way though (no hierarchy in them)
        same = false;
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep("._-");
        tokenizer a_tokens(a, sep);
        tokenizer b_tokens(b, sep);
        tokenizer::iterator tok_iter_b = b_tokens.begin();

        for (tokenizer::iterator tok_iter_a = a_tokens.begin(); tok_iter_a != a_tokens.end(); ++tok_iter_a)
        {
            if (b_tokens.end() == tok_iter_b) { a_older_than_b = false; return; } // b has no more tokens, then a is more recent

            u32 a_token = boost::lexical_cast<u32>(*tok_iter_a);
            u32 b_token = boost::lexical_cast<u32>(*tok_iter_b);

            if      (a_token > b_token) { a_older_than_b = false; return; }
            else if (a_token < b_token) { a_older_than_b = true;  return; }

            ++tok_iter_b;
        }

        if (b_tokens.end() == tok_iter_b) same = true;
        else                              a_older_than_b = true;
    }

    settings_map_type settings_map;
    base* old_version;
};

struct text_loader
{
    void load(std::string& text, std::map<std::string, std::string>& key_value)
    {
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> line_sep("\n\r");
        boost::char_separator<char> expr_sep("= ");
        tokenizer lines(text, line_sep);
        for (tokenizer::iterator line = lines.begin(); line != lines.end(); ++line)
        {
            tokenizer exprs(*line, expr_sep);
            tokenizer::iterator expr = exprs.begin();
            if (expr != exprs.end())
            {
                std::string key = *expr++;
                if (expr != exprs.end())
                {
                    key_value[key] = *expr;
                }
            }
        }
    }

    void save(base::settings_map_type& settings_map, std::string& text)
    {
        std::string temp;
        settings_map.get("version", temp);
        text = "version = " + temp + "\n";

        for (base::settings_map_type::iterator it = settings_map.begin();
             it != settings_map.end();
             ++it)
        {
            if (it->first == "version")
                continue;
            else
            {
                temp = it->second.to_string(it->second.value);
                text += it->first + " = " + temp + "\n";
            }
        }
    }
};

} 