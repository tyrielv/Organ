/********************************************************************
 * sysex.h v1.03
 * John Kinkennon
 * 11/2/2013
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * John Kinkennon, 4307 NE 65th Ct, Vancouver, WA 98661 USA
 * email: john@kinkennon.com
 *
 * This program uses Microchip USB software.  Refer to the included header
 * files for Microchip licensing restrictions.
 ********************************************************************/
#define PANEL_ID_LSB            0x00
#define PANEL_ID_MSB            0x00

#define MSG_TYPE_LCD            0x01
#define MSG_TYPE_STATUS_STRING  0x19    // is 0x10 in HW User Manual a typo?
#define MSG_TYPE_STATUS_FLOAT   0x1a
#define MSG_TYPE_STATUS_BOOLEAN 0x1b
#define MSG_TYPE_CONFIG         0x71

// sys-ex variable IDs
#define TRANSPOSER_SEMITONES    0x1a    // 26 - 28-bit signed int (-12 to +12)
#define IS_SETTER_MODE_ON       0x21    // 33 - bool
#define IS_SCOPE_MODE_ON        0x22    // 34 - bool
#define IS_RECORDING_AUDIO      0x23    // 35 - bool
#define IS_RECORDING_MIDI       0x24    // 36 - bool
#define IS_PLAYING_MIDI         0x25    // 37 - bool
#define IS_ORGAN_READY          0x26    // 38 - bool
#define IS_IN_ERROR_STATE       0x27    // 39 - bool

typedef union {
    unsigned char v[39];
    union {
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char panel_ID_LSB;      // LSB
            unsigned char panel_ID_MSB;      // MSB
            unsigned char color_code;
            unsigned char ascii_1;
            unsigned char ascii_2;
            unsigned char ascii_3;
            unsigned char ascii_4;
            unsigned char ascii_5;
            unsigned char ascii_6;
            unsigned char ascii_7;
            unsigned char ascii_8;
            unsigned char ascii_9;
            unsigned char ascii_10;
            unsigned char ascii_11;
            unsigned char ascii_12;
            unsigned char ascii_13;
            unsigned char ascii_14;
            unsigned char ascii_15;
            unsigned char ascii_16;
            unsigned char ascii_17;
            unsigned char ascii_18;
            unsigned char ascii_19;
            unsigned char ascii_20;
            unsigned char ascii_21;
            unsigned char ascii_22;
            unsigned char ascii_23;
            unsigned char ascii_24;
            unsigned char ascii_25;
            unsigned char ascii_26;
            unsigned char ascii_27;
            unsigned char ascii_28;
            unsigned char ascii_29;
            unsigned char ascii_30;
            unsigned char ascii_31;
            unsigned char ascii_32;
            unsigned char sysex_end;
        } s1;
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char panel_ID_LSB;
            unsigned char panel_ID_MSB;
            unsigned char color_code;
            unsigned char ascii_bytes[32];
            unsigned char sysex_end;
        } s2;
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char panel_ID_LSB;
            unsigned char panel_ID_MSB;
            unsigned char color_code;
            unsigned char ascii_line1[16];
            unsigned char ascii_line2[16];
            unsigned char sysex_end;
        } s3;
    };
} HW_Sysex_LCD_Msg_t, *p_HW_Sysex_LCD_Msg;

typedef union {
    unsigned char v[21];
    union {
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char variable_ID;
            unsigned char ansi_1;
            unsigned char ansi_2;
            unsigned char ansi_3;
            unsigned char ansi_4;
            unsigned char ansi_5;
            unsigned char ansi_6;
            unsigned char ansi_7;
            unsigned char ansi_8;
            unsigned char ansi_9;
            unsigned char ansi_10;
            unsigned char ansi_11;
            unsigned char ansi_12;
            unsigned char ansi_13;
            unsigned char ansi_14;
            unsigned char ansi_15;
            unsigned char ansi_16;
            unsigned char sysex_end;
        }s1;
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char variable_ID;
            unsigned char ansi_bytes[16];
            unsigned char sysex_end;
        }s2;
    };
} HW_Sysex_Status_String_t, *p_HW_Sysex_Status_String;

typedef union {
    unsigned char v[9];
    union {
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char variable_ID;
            unsigned char float_1;      // MSB
            unsigned char float_2;
            unsigned char float_3;
            unsigned char float_4;      // LSB
            unsigned char sysex_end;
        } s1;
        struct {
            unsigned char sysex_start;
            unsigned char manufacturer_ID;
            unsigned char message_type;
            unsigned char variable_ID;
            unsigned char float_bytes[4];
            unsigned char sysex_end;
        } s2;
    };
} HW_Sysex_Status_Float_t, *p_HW_Sysex_Status_Float;

typedef union {
    unsigned char v[6];
    struct {
        unsigned char sysex_start;
        unsigned char manufacturer_ID;
        unsigned char message_type;
        unsigned char variable_ID;
        unsigned char byte_value;   // boolean or 7-bit
        unsigned char sysex_end;
    } s;
} HW_Sysex_Status_Byte_t, *p_HW_Sysex_Status_Byte;

typedef union {
    unsigned char v[20];
    struct {
        unsigned char sysex_start;
        unsigned char manufacturer_ID;
        unsigned char message_type;
        unsigned char write_NVM;
        unsigned char keys_channel;
        unsigned char stops_channel;
        unsigned char pistons_channel;
        unsigned char pedal_channel;
        unsigned char pedal_transpose;
        unsigned char miditzer_aux_ch;
        unsigned char expression_A_ch;
        unsigned char expression_B_ch;
        unsigned char linked_expr_sw_ch;
        unsigned char linked_expr_sw_num;
        unsigned char vendor_select_sw_ch;
        unsigned char vendor_select_sw_num;
        unsigned char setting13;
        unsigned char setting14;
        unsigned char setting15;
        unsigned char sysex_end;
    } s;
} encoder_settings_t, *p_encoder_settings;
