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
#include <string>

namespace whi_rviz_plugins
{
    class BasePlugin
    {
    public:
        BasePlugin() = default;
        virtual ~BasePlugin() = default;

    public:
        virtual void initialize() = 0;
        virtual void process() = 0;
        virtual bool config(const std::string& Config) = 0;
    };
} // namespace whi_rviz_plugins
