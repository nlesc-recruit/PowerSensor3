set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_executable(psemulator src/emulator.cc src/device.cc)

include_directories(include/ ../host/include)
target_link_libraries(psemulator PowerSensor util Threads::Threads)

install(TARGETS psemulator DESTINATION bin)
