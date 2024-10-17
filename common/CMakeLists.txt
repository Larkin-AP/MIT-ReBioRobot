# Source/Include files
include_directories(".")
include_directories("include/" )                 # common includes
include_directories("/usr/local/include/lcm/")   # lcm includes
include_directories("../lcm-types/cpp/")
include_directories("../third-party/inih")       # inih includes
include_directories("../third-party/osqp/include")       # osqp includes
include_directories("../third-party/ParamHandler")
include_directories("/usr/local/include/eigen3")   # lcm includes
include_directories("FootstepPlanner")
include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})
file(GLOB_RECURSE sources "src/*.cpp")                   # common library cpp files
# 通过 file(GLOB_RECURSE sources "src/*.cpp") 命令，项目的所有 .cpp 文件（递归地搜索 src/ 目录下的子目录）都被收集起来，用于生成库。

# Library
add_library(biomimetics SHARED ${sources})       # produce a library used by sim/robot
# 命令创建了一个名为 biomimetics 的共享库，库的源文件来自上一步收集到的 src/ 目录中的所有 .cpp 文件。

target_link_libraries(biomimetics inih dynacore_param_handler JCQP osqp)
# 指定了库 biomimetics 依赖的其他库：

if(CMAKE_SYSTEM_NAME MATCHES Linux)
# Pull in Google Test
include(CTest)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()
include(DownloadProject.cmake)
download_project(PROJ                googletest
                 GIT_REPOSITORY      https://github.com/google/googletest.git
                 GIT_TAG             main # 原来是master，因为gtest的主分支从master改为了main，所以改为main即可通过编译，但是要确保可以连接上gtest
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
                 QUIET
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR})
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
                        "${gmock_SOURCE_DIR}/include")
endif()

# Test
file(GLOB_RECURSE test_sources "test/test_*.cpp")             # test cpp files
# 递归地搜索 test/ 目录下所有以 test_ 开头的 .cpp 文件，这些文件被视为测试代码。

add_executable(test-common ${test_sources})
# 使用测试源文件创建一个名为 test-common 的可执行文件。

target_link_libraries(test-common gtest gmock_main lcm rt inih osqp dynacore_param_handler pthread biomimetics)
target_link_libraries(test-common Goldfarb_Optimizer)
target_link_libraries(test-common JCQP)


add_test(NAME example_test COMMAND test-common)
# 定义了一个名为 example_test 的测试，运行 test-common 可执行文件。
# example_test 是一个 测试，它是通过 CTest 测试框架进行管理的。

endif(CMAKE_SYSTEM_NAME MATCHES Linux)

# Our libraries
add_subdirectory(FootstepPlanner)

# 增加gdb调试条件
set(CMAKE_BUILD_TYPE Debug)

# 复制 config 目录到 build 目录
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})


