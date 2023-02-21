/*
* Copyright (C) 2009, Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*   * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived from
*     this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

// Taken from https://github.com/robo-friends/m-explore-ros2

#ifndef TOPIC_NAME_UTILS_H
#define TOPIC_NAME_UTILS_H

#include <ctype.h>
#include <string>
#include <sstream>

namespace map_merge_2d
{
  namespace ros_names
  {
    class InvalidNameException : public std::runtime_error
    {
      public:
        InvalidNameException(const std::string& msg)
        : std::runtime_error(msg)
        {}
    };

    bool isValidCharInName(char c);
    bool validate(const std::string& name, std::string& error);
    std::string parentNamespace(const std::string& name);
    std::string clean(const std::string& name);
    std::string append(const std::string& left, const std::string& right);

  } // namespace ros_names
} // namespace map_merge_2d

#endif // TOPIC_NAME_UTILS_H