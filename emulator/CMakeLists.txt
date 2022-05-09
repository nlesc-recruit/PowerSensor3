add_executable(psemulator src/emulator.cc src/device.cc)

include_directories(include/ ../host/include)
target_link_libraries(psemulator PowerSensor)

install(TARGETS psemulator DESTINATION bin)
