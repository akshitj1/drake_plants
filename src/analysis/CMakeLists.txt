find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_executable(cubic_roa cubic_roa.cc)
target_link_libraries(cubic_roa drake::drake drake::drake-common-text-logging-gflags gflags_shared)


add_executable(roa region_of_attraction.cc)
target_link_libraries(roa drake::drake drake::drake-common-text-logging-gflags gflags_shared)

add_executable(sim_balance simulate_balance.cc)
target_link_libraries(sim_balance drake::drake drake::drake-common-text-logging-gflags gflags_shared)
