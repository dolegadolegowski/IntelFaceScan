// License: Apache 2.0. See LICENSE file in root directory.
//
// Process hardening for the helper, which may run as root on behalf of an unprivileged app (docs/HELPER.md).

#pragma once

namespace ifs
{

// Applied first thing in main(): umask 077, standard descriptors open (stdin from /dev/null), every inherited
// descriptor above 2 closed, and a minimal fixed environment (PATH, HOME of the effective user) so that no
// behavior depends on what the launching process passed down.
void harden_process();

}  // namespace ifs
