include_directories(Include)

include_directories("../../common/include/")
find_package(Threads REQUIRED)  # new added

add_library(lord_imu SHARED
        Source/mip.c
        Source/mip_sdk_3dm.c
        Source/mip_sdk_ahrs.c
        Source/mip_sdk_base.c
        Source/mip_sdk_filter.c
        Source/mip_sdk_gps.c
        Source/mip_sdk_inteface.c
        Source/mip_sdk_system.c
        Source/ring_buffer.c
        Source/mip_sdk_user_functions.c
        Source/byteswap_utilities.c
        imu_receiver.cpp
        LordImu.cpp)
        #imu_receiver.cpp

target_link_libraries(lord_imu PRIVATE Threads::Threads) # new added

#add_executable(test_imu test/lord_test.cpp)
add_executable(test_imu test/wheeltech_test.cpp)

target_link_libraries(test_imu lord_imu)
