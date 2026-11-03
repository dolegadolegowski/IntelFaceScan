// License: Apache 2.0. See LICENSE file in root directory.

#include "ifs_protocol.h"

#include <stddef.h>

int ifs_wire_header_valid(const ifs_wire_header *header)
{
    if (header == NULL)
        return -1;
    if (header->magic != IFS_PROTOCOL_MAGIC || header->version != IFS_PROTOCOL_VERSION)
        return -1;
    if (header->json_bytes > IFS_MAX_JSON_BYTES || header->payload_bytes > IFS_MAX_PAYLOAD_BYTES)
        return -1;
    if (header->flags & ~(uint32_t)IFS_FLAG_HAS_FD)
        return -1;
    return 0;
}

void ifs_wire_header_init(ifs_wire_header *header, uint16_t type, uint32_t json_bytes, uint64_t payload_bytes,
                          uint32_t flags)
{
    header->magic = IFS_PROTOCOL_MAGIC;
    header->version = (uint16_t)IFS_PROTOCOL_VERSION;
    header->type = type;
    header->json_bytes = json_bytes;
    header->flags = flags;
    header->payload_bytes = payload_bytes;
}
