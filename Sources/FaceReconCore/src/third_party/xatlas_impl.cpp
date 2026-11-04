// License: Apache 2.0. See LICENSE file in root directory.
//
// Unity wrapper: compiles xatlas (MIT, third_party/xatlas) into FaceReconCore. Its warnings are not ours to fix.

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#include "xatlas.cpp"
#pragma clang diagnostic pop
