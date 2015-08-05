/* 
 * File:   midi.h
 * Author: John Kinkennon
 *
 * Created on June 4, 2012, 12:13 PM
 */

#ifndef MIDI_H
#define	MIDI_H

#ifdef	__cplusplus
extern "C" {
#endif

/** MIDI Default Values ********************************************/
#define M_VELOCITY_ON       0x60    // default where no touch sensitivity
#define M_VELOCITY_OFF      0x00    // default for note off msg
#define M_KBD_FIRST_KEY     0x24    // first key on a 61 key kybd

/** MIDI Channel Voice Messages ************************************/
#define M_NOTE_OFF      0b10000000  // 0x8n - where n is the channel
#define M_NOTE_ON       0b10010000  // 0x9n -	(MIDI ch 1 is n=0)
#define M_AFTERTOUCH	0b10100000  // 0xAn - polyphonic key pressure
#define M_CTRL_CHANGE	0b10110000  // 0xBn - control change
#define M_PROG_CHANGE	0b11000000  // 0xCn - program change
#define M_CH_PRESSURE	0b11010000  // 0xDn - also called aftertouch
#define M_PITCH_WHEEL	0b11100000  // 0xEn - pitch wheel change

/** MIDI Channel Mode Messages - only for CC's with channel > 119 **/
#define M_ALL_SOUND_OFF 0b01111000  // 0x78 - all sound off
#define M_RESET_ALL_C	0b01111001  // 0x79 - reset all controllers
#define M_LOCAL_CONTROL 0b01111010  // 0x7A - local control (0=OFF, 127=ON)
#define M_ALL_NOTES_OFF 0b01111011  // 0x7B - all notes off
#define M_OMNI_OFF      0b01111100  // 0x7C - omni mode off
#define M_OMNI_ON       0b01111101  // 0x7D - omin mode on
#define M_MONO_ON       0b01111110  // 0x7E - mono mode on
#define M_POLY_ON       0b01111111  // 0x7F - poly mode on

/** MIDI System Common Messages ************************************/
#define M_SYSTEM_EX     0b11110000  // 0xF0 - system exclusive
#define M_SYSEX_ID      0b01111101  // 0x7D - test or development id
#define M_TIME_CODE_QF	0b11110001  // 0xF1 - time code quarter frame
#define M_SONG_POS_PTR	0b11110010  // 0xF2 - song position pointer
#define M_SONG_SELECT	0b11110011  // 0xF3 - song select
#define M_TUNE_REQUEST	0b11110110  // 0xF6 - tune request (tune osc's)
#define M_END_EXCLUSIVE 0b11110111  // 0xF7 - end of exclusive (see F0)

/** MIDI System Real-Time Messages *********************************/
#define M_TIMING_CLOCK	0b11111000  // 0xF8 - timing clock
#define M_START         0b11111010  // 0xFA - start
#define M_CONTINUE      0b11111011  // 0xFB - continue
#define M_STOP          0b11111100  // 0xFC - stop
#define M_ACTIVE_SENSE	0b11111110  // 0xFE - active sensing
#define M_RESET         0b11111111  // 0xFF - reset

/** MIDI Channel Control Messages **********************************/
#define M_CC_BANK_SEL	0b00000000  // 0x00 - bank select
#define M_CC_MOD_WHEEL	0b00000001  // 0x01 - modulation wheel
#define M_CC_BREATH_CTL	0b00000010  // 0x02 - breath controller
#define M_CC_FOOT_CTL	0b00000100  // 0x04 - foot controller
#define M_CC_PORTAMENTO	0b00000101  // 0x05 - portamento
#define M_CC_VOLUME     0b00000111  // 0x07 - ch volume
#define M_CC_BALANCE	0b00001000  // 0x08 - balance
#define M_CC_PAN        0b00001010  // 0x0A - pan
#define M_CC_EXPRESSION	0b00001011  // 0x0B - expression

#ifdef	__cplusplus
}
#endif

#endif	/* MIDI_H */

