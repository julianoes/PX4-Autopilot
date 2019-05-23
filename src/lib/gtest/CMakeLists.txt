############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_definitions(
	-DGTEST_HAS_POSIX_RE=0
	-DGTEST_HAS_CLONE=0
	-DGTEST_HAS_EXCEPTIONS=0
	-DGTEST_HAS_TR1_TUPLE=0
	-DGTEST_HAS_STREAM_REDIRECTION=0
	-DGTEST_HAS_RTTI=0
	-DGTEST_HAS_CXXABI_H_=0
)

add_subdirectory(googletest)

get_target_property(GTEST_COMPILE_FLAGS gtest COMPILE_OPTIONS)
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_POSIX_RE=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_CLONE=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_EXCEPTIONS=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_TR1_TUPLE=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_STREAM_REDIRECTION=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_RTTI=0")
list(APPEND GTEST_COMPILE_FLAGS "-DGTEST_HAS_CXXABI_H_=0")
list(REMOVE_ITEM GTEST_COMPILE_FLAGS "-include")
list(REMOVE_ITEM GTEST_COMPILE_FLAGS "visibility.h")
set_target_properties(gtest PROPERTIES COMPILE_OPTIONS "${GTEST_COMPILE_FLAGS}")

add_dependencies(gtest prebuild_targets)
