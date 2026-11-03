// License: Apache 2.0. See LICENSE file in root directory.
//
// PLACEHOLDER (owner: tests/data module) - command line tool: reconstruct from recordings/datasets, synthetic data,
// evaluation. Usage: ifscan <reconstruct|synth|eval|info> ...

import Foundation

FileHandle.standardError.write("usage: ifscan <reconstruct|synth|eval|info> ...\n".data(using: .utf8)!)
exit(64)
