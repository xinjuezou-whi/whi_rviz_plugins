/******************************************************************
abstract base class for sub plugins

Features:
- virtual interface
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-09-20: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <yaml-cpp/yaml.h>
#include <string>

namespace whi_rviz_plugins
{
    class BasePlugin
    {
    public:
        BasePlugin() = default;
        virtual ~BasePlugin() = default;

    public:
        virtual void initialize(const YAML::Node& Node) = 0;
        virtual void process(const std::string& Task, void* Data = nullptr, std::size_t Len = 0) = 0;
        virtual bool addTask(const std::string& Task) = 0;
        virtual void abort() = 0;
    };
} // namespace whi_rviz_plugins
