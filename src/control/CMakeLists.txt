add_library(lqr_tree SHARED lqr_tree.h lqr_tree.cc)
target_link_libraries(lqr_tree drake::drake)

add_library(lqr SHARED lqr.h)
target_link_libraries(lqr drake::drake)
