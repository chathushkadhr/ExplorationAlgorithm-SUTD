#include <map_merge_2d/util/topic_name_utils.hpp>

using namespace map_merge_2d;

bool ros_names::isValidCharInName(char c)
{
    if (isalnum(c) || c == '/' || c == '_')
    {
    return true;
    }

    return false;
}

bool ros_names::validate(const std::string& name, std::string& error)
{
    if (name.empty())
    {
    return true;
    }

    // First element is special, can be only ~ / or alpha
    char c = name[0];
    if (!isalpha(c) && c != '/' && c != '~')
    {
    std::stringstream ss;
    ss << "Character [" << c << "] is not valid as the first character in Graph Resource Name [" << name << "].  Valid characters are a-z, A-Z, / and in some cases ~.";
    error = ss.str();
    return false;
    }

    for (size_t i = 1; i < name.size(); ++i)
    {
    c = name[i];
    if (!isValidCharInName(c))
    {
        std::stringstream ss;
        ss << "Character [" << c << "] at element [" << i << "] is not valid in Graph Resource Name [" << name <<"].  Valid characters are a-z, A-Z, 0-9, / and _.";
        error = ss.str();

        return false;
    }
    }

    return true;
}

std::string ros_names::parentNamespace(const std::string& name)
{
    std::string error;
    if (!validate(name, error))
    {
        throw InvalidNameException(error);
    }

    if (!name.compare(""))  return "";
    if (!name.compare("/")) return "/"; 

    std::string stripped_name;

    // rstrip trailing slash
    if (name.find_last_of('/') == name.size()-1)
    stripped_name = name.substr(0, name.size() -2);
    else
    stripped_name = name;

    //pull everything up to the last /
    size_t last_pos = stripped_name.find_last_of('/');
    if (last_pos == std::string::npos)
    {
    return "";
    }
    else if (last_pos == 0)
    return "/";
    return stripped_name.substr(0, last_pos);
}

std::string ros_names::clean(const std::string& name)
{
    std::string clean = name;

    size_t pos = clean.find("//");
    while (pos != std::string::npos)
    {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
    }

    if (*clean.rbegin() == '/')
    {
    clean.erase(clean.size() - 1, 1);
    }

    return clean;
}

std::string ros_names::append(const std::string& left, const std::string& right)
{
    return clean(left + "/" + right);
}
