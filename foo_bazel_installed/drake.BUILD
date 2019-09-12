cc_library(
    name = "eigen3",
    hdrs = glob(["include/eigen3/**"]),
    strip_include_prefix = "include/eigen3",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "stx",
    hdrs = glob(["include/stx/**/*.hpp"]),
    strip_include_prefix = "include/stx",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "fmt",
    hdrs = glob(["include/fmt/**/*.h"]),
    strip_include_prefix = "include/fmt",
    visibility = ["//visibility:public"],
)


cc_library(
    name = "drake-lib",
    srcs = ["lib/libdrake.so"],
    hdrs = glob(["include/drake/**/*.h"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = ["eigen3", "stx","fmt"],
    linkstatic = 0,
)
