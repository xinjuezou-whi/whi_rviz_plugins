/******************************************************************
utilities for rviz2 plugins

Features:
- pipe execute
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2028-08-27: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <string>
#include <iostream>

namespace whi_rviz_plugins
{
    static std::vector<std::string> pipeExecute(const char* Cmd)
    {
        std::vector<std::string> results;

        const size_t BUF_LEN = 512;
        char buf[BUF_LEN] = {0};

        // Force line-buffered output and capture stderr too
        std::string fullCmd = std::string("stdbuf -oL ") + Cmd + " 2>&1";

        FILE* pipe = popen(fullCmd.c_str(), "r");
        if (!pipe)
        {
            perror("popen failed");
            return results;
        }

        while (fgets(buf, BUF_LEN, pipe) != NULL)
        {
            std::string line(buf);

            // remove trailing newline safely
            if (!line.empty() && line.back() == '\n')
            {
                line.pop_back();
            }

            std::cout << "pipe read line: " << line << std::endl;
            results.push_back(line);
        }
        pclose(pipe);

        return results;
    }
} // end namespace whi_rviz_plugins
