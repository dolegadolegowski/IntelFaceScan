// License: Apache 2.0. See LICENSE file in root directory.
//
// Wire protocol between the privileged camera helper (C++, runs as root, owns the RealSense camera) and the
// unprivileged IntelFaceScan app (Swift). Shared by both sides; see docs/PROTOCOL.md for the JSON schemas.
//
// Transport: a local (AF_UNIX, SOCK_STREAM) socket. The app listens, the helper connects. Every message is
//
//     ifs_wire_header (24 bytes, little endian) | JSON (json_bytes, UTF-8, no NUL) | payload (payload_bytes)
//
// The JSON object carries the control data; the binary payload carries pixel planes (FRAMESET) only. A message
// may carry one file descriptor as SCM_RIGHTS ancillary data on its first byte (IFS_FLAG_HAS_FD), used only to
// hand the helper a file the app opened as the user (recordings): the root helper never opens paths the app
// names.

#ifndef IFS_PROTOCOL_H
#define IFS_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IFS_PROTOCOL_MAGIC   0x31534649u          /* "IFS1" as little-endian bytes */
#define IFS_PROTOCOL_VERSION 1u

#define IFS_MAX_JSON_BYTES    (256u * 1024u)        /* larger control messages are a protocol error */
#define IFS_MAX_PAYLOAD_BYTES (64u * 1024u * 1024u) /* larger payloads are a protocol error */

/* ifs_wire_header.flags */
#define IFS_FLAG_HAS_FD 0x1u

typedef enum ifs_msg_type {
    /* helper -> app */
    IFS_MSG_HELLO       = 1,   /* first message after connecting: version, pid, mode, euid */
    IFS_MSG_REPLY       = 2,   /* reply to a command: {"request_id", "ok", "error"?, "data"?} */
    IFS_MSG_FRAMESET    = 3,   /* one synchronized frameset: JSON metadata + pixel planes in the payload */
    IFS_MSG_EVENT       = 4,   /* asynchronous event: device lost/found, stream restarted, recording finished, ... */
    IFS_MSG_LOG         = 5,   /* log line: {"level": "info|warning|error|debug", "message"} */
    IFS_MSG_BYE         = 6,   /* sent right before the helper exits, after the camera was handed back */

    /* app -> helper (all carry "request_id"; the helper answers each with exactly one IFS_MSG_REPLY) */
    IFS_CMD_QUERY_DEVICE   = 100, /* -> data: DeviceInfo */
    IFS_CMD_CONFIGURE      = 101, /* CaptureConfiguration; applied now if streaming (may restart the streams) */
    IFS_CMD_START          = 102, /* start streaming with the current configuration */
    IFS_CMD_STOP           = 103, /* stop streaming */
    IFS_CMD_SET_OPTION     = 104, /* {"sensor": "depth|color", "option": "<rs2 option name>", "value": number} */
    IFS_CMD_SET_ROI        = 105, /* auto-exposure ROI in color pixels: {"x","y","width","height","distance_m"?} */
    IFS_CMD_SET_FILTERS    = 106, /* FilterConfiguration, applied to the next frameset */
    IFS_CMD_RECORD_START   = 107, /* carries the destination fd (IFS_FLAG_HAS_FD): {"format": "db3|bag"} */
    IFS_CMD_RECORD_STOP    = 108, /* finishes the recording and copies it into the fd; -> data: {"bytes"} */
    IFS_CMD_PLAYBACK       = 109, /* playback mode only: {"action": "pause|resume", "real_time": bool} */
    IFS_CMD_HARDWARE_RESET = 110, /* resets the camera (it re-enumerates; the helper reconnects) */
    IFS_CMD_QUIT           = 111  /* stop, hand the camera back to macOS, send BYE and exit */
} ifs_msg_type;

typedef struct ifs_wire_header {
    uint32_t magic;          /* IFS_PROTOCOL_MAGIC */
    uint16_t version;        /* IFS_PROTOCOL_VERSION */
    uint16_t type;           /* ifs_msg_type */
    uint32_t json_bytes;     /* <= IFS_MAX_JSON_BYTES */
    uint32_t flags;          /* IFS_FLAG_* */
    uint64_t payload_bytes;  /* <= IFS_MAX_PAYLOAD_BYTES */
} ifs_wire_header;

#if defined(__cplusplus)
static_assert(sizeof(ifs_wire_header) == 24, "ifs_wire_header must be 24 bytes");
#else
_Static_assert(sizeof(ifs_wire_header) == 24, "ifs_wire_header must be 24 bytes");
#endif

/* Pixel formats of FRAMESET planes ("format" in the plane JSON) */
#define IFS_FORMAT_RGB8 "rgb8"   /* 3 bytes per pixel, R G B */
#define IFS_FORMAT_Z16  "z16"    /* uint16 little endian, multiply by depth_scale for meters, 0 = no data */
#define IFS_FORMAT_Y8   "y8"     /* 8-bit infrared */

/* Plane kinds of FRAMESET planes ("kind" in the plane JSON) */
#define IFS_PLANE_COLOR         "color"          /* RGB camera, color intrinsics */
#define IFS_PLANE_DEPTH_ALIGNED "depth_aligned"  /* filtered depth re-projected onto the color image (same size and
                                                    intrinsics as color) - the input of the reconstruction */
#define IFS_PLANE_DEPTH         "depth"          /* filtered depth in the depth sensor's own frame (optional) */
#define IFS_PLANE_IR_LEFT       "ir_left"        /* left infrared (optional, diagnostics) */

/* Validates a received header; returns 0 when it is acceptable, -1 otherwise. */
int ifs_wire_header_valid(const ifs_wire_header *header);

/* Fills a header for sending. */
void ifs_wire_header_init(ifs_wire_header *header, uint16_t type, uint32_t json_bytes, uint64_t payload_bytes,
                          uint32_t flags);

#ifdef __cplusplus
}
#endif

#endif /* IFS_PROTOCOL_H */
