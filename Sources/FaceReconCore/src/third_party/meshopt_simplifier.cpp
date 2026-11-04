// License: Apache 2.0. See LICENSE file in root directory.
//
// Unity wrapper: compiles the meshoptimizer simplifier (MIT, third_party/meshoptimizer) into FaceReconCore. One
// wrapper per meshoptimizer source keeps their file-local helpers from colliding.

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include "simplifier.cpp"
#pragma clang diagnostic pop
