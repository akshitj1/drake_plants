find_package(GFlags MODULE REQUIRED COMPONENTS shared)

add_library(glider SHARED glider.cc glider.h)
target_link_libraries(glider drake::drake)

add_library(tvlqr SHARED tvlqr.h tvlqr.cc)
target_link_libraries(tvlqr drake::drake)

add_executable(simulate_glider simulate.cc)
target_link_libraries(simulate_glider glider tvlqr drake::drake-common-text-logging-gflags gflags_shared)


#add_test(NAME simulate
#        COMMAND simulate
#        )

