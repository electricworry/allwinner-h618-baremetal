#include <stdint.h>
#include <stdbool.h>
#include "util.h"

static uint8_t edid_data[] = {
    0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x41, 0x0c, 0xe1, 0x08, 0x11, 0x04, 0x00, 0x00,
    0x19, 0x19, 0x01, 0x03, 0x80, 0x58, 0x31, 0x78, 0x2a, 0xf6, 0x3d, 0xa3, 0x55, 0x4e, 0x9e, 0x27,
    0x0d, 0x47, 0x4a, 0xbd, 0x4b, 0x00, 0xd1, 0xc0, 0x81, 0x80, 0x81, 0x40, 0x95, 0x0f, 0x95, 0x00,
    0xb3, 0x00, 0x81, 0xc0, 0x01, 0x01, 0x04, 0x74, 0x00, 0x30, 0xf2, 0x70, 0x5a, 0x80, 0xb0, 0x58,
    0x8a, 0x00, 0x6e, 0xe5, 0x31, 0x00, 0x00, 0x1a, 0x56, 0x5e, 0x00, 0xa0, 0xa0, 0xa0, 0x29, 0x50,
    0x30, 0x20, 0x35, 0x00, 0x6e, 0xe5, 0x31, 0x00, 0x00, 0x1e, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x50,
    0x48, 0x4c, 0x20, 0x42, 0x44, 0x4d, 0x34, 0x30, 0x36, 0x35, 0x0a, 0x20, 0x00, 0x00, 0x00, 0xfd,
    0x00, 0x17, 0x50, 0x1e, 0x63, 0x1e, 0x00, 0x0a, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x01, 0x92,
};

enum {
    EDID_OK = 0,
    EDID_UNSUPPORTED,
    EDID_FATAL,
    EDID_VER_1_REV_4,
    EDID_VER_1_REV_3,
    EDID_COLOUR_DEPTH_UNDEFINED,
    EDID_COLOUR_DEPTH_6,
    EDID_COLOUR_DEPTH_8,
    EDID_COLOUR_DEPTH_10,
    EDID_COLOUR_DEPTH_12,
    EDID_COLOUR_DEPTH_14,
    EDID_COLOUR_DEPTH_16,
    EDID_COLOUR_TYPE_RGB444,
    EDID_COLOUR_TYPE_RGB444_YCRCB444,
    EDID_COLOUR_TYPE_RGB444_YCRCB422,
    EDID_COLOUR_TYPE_RGB444_YCRCB444_YCRCB422,
    EDID_D_COMPOSITE_SYNC,
    EDID_D_SEPARATE_SYNC
};

struct edid {
    /* EDID structure version */
    uint8_t  ver;
    uint8_t  rev;

    /* Vendor & Product Identification */
    char manufacturer[4];
    uint16_t product_code;
    uint32_t serial_number;
    uint8_t  week;
    uint16_t year;

    /* Basic Display Parameters & Features */
    uint8_t  colour_depth;
    bool     size_is_aspect;
    uint8_t  width_cm;
    uint8_t  height_cm;
    float    aspect_ratio;
    bool     gamma_elsewhere;
    float    gamma;
    uint8_t  colour_type;

    /* Colour Characteristics */
    double red_x;
    double red_y;
    double green_x;
    double green_y;
    double blue_x;
    double blue_y;
    double white_x;
    double white_y;

    /* Data from 18-byte descriptiors */
    uint8_t display_product_name[14];
    uint8_t display_product_serial[14];
    uint8_t string_descriptor[14];

    /* Preferred DTD */
    uint32_t pixel_clock;
    uint16_t horizontal_px;
    uint16_t horizontal_blanking_px;
    uint16_t horizontal_front_porch_px;
    uint16_t horizontal_sync_pulse_px;
    uint16_t horizontal_back_porch_px;
    uint16_t horizontal_mm;
    uint16_t vertical_px;
    uint16_t vertical_blanking_px;
    uint16_t vertical_front_porch_px;
    uint16_t vertical_sync_pulse_px;
    uint16_t vertical_back_porch_px;
    uint16_t vertical_mm;
    uint8_t  horizontal_border_px; // A value here means that many px*2 (left/right)
    uint8_t  vertical_border_px;   // A value here means that many px*2 (top/bottom)
    bool     is_interlaced;
    // uint8_t  sync_type;
    // bool     composite_with_serrations;
    // bool     h_sync_is_positive;
    // bool     v_sync_is_positive;

    /* Extension blocks */
    uint8_t  extension_block_count;
};


int parse_edid_structure(uint8_t *edid, struct edid *dst);
void print_edid(struct edid *e);