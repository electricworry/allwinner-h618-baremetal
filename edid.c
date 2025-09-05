#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "edid.h"

void copy_lf_terminated_string(uint8_t *dst, int dstlen, uint8_t *src, int srclen)
{
    for (int i = 0; i < MIN(dstlen, srclen); i++)
    {
        uint8_t c = src[i];
        if (c == 0x0a) {
            break;
        }
        dst[i] = c;
    }
}

void pnpid_to_str(uint8_t *dst, uint16_t src)
{
    dst[0] = ((GENMASK(14, 10) & src) >> 10) + '@';
    dst[1] = ((GENMASK(9, 5)   & src) >> 5)  + '@';
    dst[2] = ((GENMASK(4, 0)   & src) >> 0)  + '@';
}

double chromacity_bin_to_double(uint16_t n)
{
    double res = 0.0;
    for (int i = 0; i < 10; i++) {
        if (BIT(i) & n) {
            res += pow(2, (i - 10));
        }
    }
    return res;
}

int parse_edid_structure(uint8_t *edid, struct edid *dst)
{
    uint8_t ver;
    uint8_t val;
    uint8_t offset;

    /* Validate header */
    if (edid[0] != 0 || edid[7] != 0)
        return EDID_FATAL;
    for (int i = 1; i < 7; i++) {
        if (edid[i] != 0xff)
            return EDID_FATAL;
    }

    /* EDID version/revision */
    dst->ver = edid[0x12];
    dst->rev = edid[0x13];
    if (dst->ver != 1)
        return EDID_FATAL;
    switch (dst->rev) {
        case 4:
            ver = EDID_VER_1_REV_4;
            break;
        case 3:
            ver = EDID_VER_1_REV_3;
            break;
        default:
            return EDID_FATAL;
    }
    dst->extension_block_count = edid[0x7e];

    /* Checksum - Only mandatory in EDID 1.4 */
    if (dst->ver == 1 && dst->rev == 4) {
        val = 0;
        for (offset = 0; offset < 0x80; offset++) {
            val += edid[offset];
            if (val != 0)
                return EDID_FATAL;
        }
    }

    /* Vendor & Product Identification */
    pnpid_to_str(dst->manufacturer, (edid[0x08] << 8) | edid[0x09]);
    dst->product_code = edid[0x0a] | ((uint16_t)(edid[0x0b]) << 8);
    dst->serial_number = edid[0x0c] | ((uint32_t)(edid[0x0d]) << 8) |
        ((uint32_t)(edid[0x0e]) << 16) | ((uint32_t)(edid[0x0f]) << 16);
    if (edid[0x10] > 0 && edid[0x10] <= 54)
        dst->week = edid[0x10];
    if (edid[0x10] > 0x0f)
        dst->year = edid[0x10] + 1990;

    /* Basic Display Parameters & Features */
    if (! (edid[0x14] & BIT(7)) ) // We don't support analogue
        return EDID_UNSUPPORTED;
    val = (GENMASK(6, 4) & edid[0x14]) >> 4;
    if (val == 7)
        return EDID_FATAL;
    dst->colour_depth = EDID_COLOUR_DEPTH_UNDEFINED + val;
    /* Interface standard: DVI, HDMI, DP, etc. We don't care. */
    
    if (!edid[0x16]) {
        if (edid[0x15]) {
            // Value is aspect ratio landscape
            dst->size_is_aspect = true;
            dst->aspect_ratio = ((float)edid[0x15] + 99.0) / 100.0;
        }
    }
    else if (!edid[0x15]) {
        // Value is aspect ratio portrait
        dst->size_is_aspect = true;
        dst->aspect_ratio = 100.0 / ((float)edid[0x15] + 99.0);
    }
    else {
        // size in cm
        dst->width_cm = edid[0x15];
        dst->height_cm = edid[0x16];
        dst->aspect_ratio = (float) dst->width_cm / (float) dst->height_cm;
    }
    if (!edid[0x17])
        return EDID_FATAL;
    else if (!edid[0x17] == 0xff)
        dst->gamma_elsewhere = true;
    else
        dst->gamma = ((float)edid[0x17] + 100.0) / 100.0;
    val = (GENMASK(4, 3) & edid[0x18]) >> 3;
    dst->colour_type = EDID_COLOUR_TYPE_RGB444 + val;
    /* TODO: in EDID 1.4 if 0x18:BIT(1) == 0 this means preferred DTD doesn't
       include native pixel format and preferred refresh rate. */

    /* Colour Characteristics */
    dst->red_x = chromacity_bin_to_double( ((uint16_t)edid[0x1b]) << 2
        | ((GENMASK(7, 6) & edid[0x19]) >> 6 ) );
    dst->red_y = chromacity_bin_to_double( ((uint16_t)edid[0x1c]) << 2
        | ((GENMASK(5, 4) & edid[0x19]) >> 4 ) );
    dst->green_x = chromacity_bin_to_double( ((uint16_t)edid[0x1d]) << 2
        | ((GENMASK(3, 2) & edid[0x19]) >> 2 ) );
    dst->green_y = chromacity_bin_to_double( ((uint16_t)edid[0x1e]) << 2
        | ((GENMASK(1, 0) & edid[0x19]) >> 0 ) );
    dst->blue_x = chromacity_bin_to_double( ((uint16_t)edid[0x1f]) << 2
        | ((GENMASK(7, 6) & edid[0x1a]) >> 6 ) );
    dst->blue_y = chromacity_bin_to_double( ((uint16_t)edid[0x20]) << 2
        | ((GENMASK(5, 4) & edid[0x1a]) >> 4 ) );
    dst->white_x = chromacity_bin_to_double( ((uint16_t)edid[0x21]) << 2
        | ((GENMASK(3, 2) & edid[0x1a]) >> 2 ) );
    dst->white_y = chromacity_bin_to_double( ((uint16_t)edid[0x22]) << 2
        | ((GENMASK(1, 0) & edid[0x1a]) >> 0 ) );

    /* 18-byte descriptors */
    for (offset = 0x36; offset < 0x7e; offset += 0x12)
    {
        // Display Descriptors
        if (edid[offset] == 0x00 &&
            edid[offset + 1] == 0x00
        ) {
            if (offset == 0x36) {
                return EDID_FATAL;
            }

            if (edid[offset + 2] == 0x00) {
                /* Display Product Serial */
                if (edid[offset + 3] == 0xff &&
                    edid[offset + 4] == 0x00
                ) {
                    copy_lf_terminated_string(dst->display_product_name, 13, &edid[offset + 5], 13);
                }
                /* Alphanumeric Data String Descriptior */
                else if (edid[offset + 3] == 0xfe &&
                    edid[offset + 4] == 0x00
                ) {
                    copy_lf_terminated_string(dst->string_descriptor, 13, &edid[offset + 5], 13);
                }
                /* Display Product Name */
                else if (edid[offset + 3] == 0xfc &&
                    edid[offset + 4] == 0x00
                ) {
                    copy_lf_terminated_string(dst->display_product_name, 13, &edid[offset + 5], 13);
                }
                /* TODO: Colour Point Descriptor Definition */
                /* Ignoring range limits, standard/established timings, DCM, CVT, etc.  */
            }
                
        }
        else {
            if (offset != 0x36) {
                // We just care about the preferred DTD
                break;
            }
            // (Preferred) Detailed Timing Definition
            dst->pixel_clock = (edid[offset] |
                (edid[offset + 1] << 8)) * 10000;
            dst->horizontal_px = (edid[offset + 2] |
                ((GENMASK(7, 4) & edid[offset + 4]) << 4));
            dst->horizontal_blanking_px = (edid[offset + 3] |
                ((GENMASK(3, 0) & edid[offset + 4]) << 8));
            dst->vertical_px = (edid[offset + 5] |
                ((GENMASK(7, 4) & edid[offset + 7]) << 4));
            dst->vertical_blanking_px = (edid[offset + 6] |
                ((GENMASK(3, 0) & edid[offset + 7]) << 8));
            dst->horizontal_front_porch_px = (edid[offset + 8] |
                ((GENMASK(7, 6) & edid[offset + 11]) << 2));
            dst->horizontal_sync_pulse_px = (edid[offset + 9] |
                ((GENMASK(5, 4) & edid[offset + 11]) << 4));
            dst->vertical_front_porch_px = (
                ((GENMASK(7, 4) & edid[offset + 10]) >> 4) |
                ((GENMASK(3, 2) & edid[offset + 11]) << 6));
            dst->vertical_sync_pulse_px = (
                ((GENMASK(3, 0) & edid[offset + 10])) |
                ((GENMASK(1, 0) & edid[offset + 11]) << 8));
            if ((dst->horizontal_front_porch_px + dst->horizontal_sync_pulse_px) >
                dst->horizontal_blanking_px ||
                (dst->horizontal_front_porch_px + dst->horizontal_sync_pulse_px) >
                dst->horizontal_blanking_px)
            {
                return EDID_FATAL;
            }
            dst->horizontal_back_porch_px = dst->horizontal_blanking_px -
                dst->horizontal_front_porch_px - dst->horizontal_sync_pulse_px;
            dst->vertical_back_porch_px = dst->vertical_blanking_px -
                dst->vertical_front_porch_px - dst->vertical_sync_pulse_px;
            dst->horizontal_mm = (edid[offset + 12] |
                ((GENMASK(7, 4) & edid[offset + 14]) << 4));
            dst->vertical_mm = (edid[offset + 13] |
                ((GENMASK(3, 0) & edid[offset + 14]) << 8));
            dst->horizontal_border = edid[offset + 15];
            dst->vertical_border = edid[offset + 16];
            dst->is_interlaced = BIT(7) & edid[offset + 17];

            /* I think the rest of the preferred DTD can be ignored */
        }
    }
    
    return EDID_OK;
}

void print_edid(struct edid *e)
{
    printf("===== START EDID =====\n");
    printf("EDID Ver/Rev: %d.%d\n", e->ver, e->rev);
    printf("Manufacturer %s\n", e->manufacturer);
    printf("Product code: 0x%04x\n", e->product_code);
    printf("Serial number: 0x%08x\n", e->serial_number);
    printf("Week & Year: %d %d\n", e->week, e->year);
    printf("Colour depth: ");
    switch (e->colour_depth) {
        case EDID_COLOUR_DEPTH_UNDEFINED:
            printf("Undefined");
            break;
        case EDID_COLOUR_DEPTH_6:
            printf("6bpp");
            break;
        case EDID_COLOUR_DEPTH_8:
            printf("8bpp");
            break;
        case EDID_COLOUR_DEPTH_10:
            printf("10bpp");
            break;
        case EDID_COLOUR_DEPTH_12:
            printf("12bpp");
            break;
        case EDID_COLOUR_DEPTH_14:
            printf("14bpp");
            break;
        case EDID_COLOUR_DEPTH_16:
            printf("16bpp");
            break;
        default:
    }
    printf("\n");
    if (e->size_is_aspect) {
        printf("Aspect ratio: %f\n", e->aspect_ratio);
    } else {
        printf("Dimensions: W %dcm H %dcm (Aspect ratio %f)\n", e->width_cm, e->height_cm, e->aspect_ratio);
    }
    printf("Gamma: %f\n", e->gamma);
    printf("Colour type: ");
    switch (e->colour_type) {
        case EDID_COLOUR_TYPE_RGB444:
            printf("RGB444");
            break;
        case EDID_COLOUR_TYPE_RGB444_YCRCB444:
            printf("RGB444_YCRCB444");
            break;
        case EDID_COLOUR_TYPE_RGB444_YCRCB422:
            printf("RGB444_YCRCB422");
            break;
        case EDID_COLOUR_TYPE_RGB444_YCRCB444_YCRCB422:
            printf("RGB444_YCRCB444_YCRCB422");
            break;
        default:
    }
    printf("\n");
    printf("Red_x:   %f Red_y:   %f\n", e->red_x,   e->red_y);
    printf("Green_x: %f Green_y: %f\n", e->green_x, e->green_y);
    printf("Blue_x:  %f Blue_y:  %f\n", e->blue_x,  e->blue_y);
    printf("White_x: %f White_y: %f\n", e->white_x, e->white_y);
    printf("Display Product Name: %s\n", e->display_product_name);
    printf("Display Product Serial: %s\n", e->display_product_serial);
    printf("String descriptor: %s\n", e->string_descriptor);
    printf("Pixel clock: %d\n", e->pixel_clock);
    printf("horizontal_px: %d\n", e->horizontal_px);
    printf("horizontal_blanking_px: %d\n", e->horizontal_blanking_px);
    printf("horizontal_front_porch_px: %d\n", e->horizontal_front_porch_px);
    printf("horizontal_sync_pulse_px: %d\n", e->horizontal_sync_pulse_px);
    printf("horizontal_back_porch_px: %d\n", e->horizontal_back_porch_px);
    printf("horizontal_mm: %d\n", e->horizontal_mm);
    printf("vertical_px: %d\n", e->vertical_px);
    printf("vertical_blanking_px: %d\n", e->vertical_blanking_px);
    printf("vertical_front_porch_px: %d\n", e->vertical_front_porch_px);
    printf("vertical_sync_pulse_px: %d\n", e->vertical_sync_pulse_px);
    printf("vertical_back_porch_px: %d\n", e->vertical_back_porch_px);
    printf("vertical_mm: %d\n", e->vertical_mm);
    printf("horizontal_border: %d\n", e->horizontal_border);
    printf("vertical_border: %d\n", e->vertical_border);
    printf("Interlaced: %d\n", e->is_interlaced);
    // printf("Hpol: %c Vpol: %c\n",
    //     e->h_sync_is_positive ? 'P' : 'N',
    //     e->v_sync_is_positive ? 'P' : 'N'
    // );
    printf("Extension block count: %d\n", e->extension_block_count);
    printf("====== END EDID ======\n");
}
