/*
 * preference_writer.c
 *
 *  Created on: Apr 13, 2020
 *      Author: ben
 */


#include "preference_writer.h"
#include "flash_writer.h"
#include "user_config.h"
/*
PreferenceWriter::PreferenceWriter(uint32_t sector) {
    writer = new FlashWriter(sector);
    __sector = sector;
    __ready = false;
}
*/

void preference_writer_init(PreferenceWriter * pr, uint32_t sector){
	flash_writer_init(&pr->fw, sector);
	pr->sector = sector;
}


void preference_writer_open(PreferenceWriter * pr) {
    flash_writer_open(&pr->fw);
    pr->ready = true;
}

bool  preference_writer_ready(PreferenceWriter pr) {
    return pr.ready;
}

void preference_writer_write_int(int x, int index) {
    __int_reg[index] = x;
}

void preference_writer_write_float(float x, int index) {
    __float_reg[index] = x;
}

void preference_writer_flush(PreferenceWriter * pr) {
    int offs;
    for (offs = 0; offs < MAX_INTS_IN_FLASH; offs++) {
        flash_writer_write_int(pr->fw, offs, __int_reg[offs]);
    }
    for (; offs < MAX_INTS_IN_FLASH+MAX_FLOATS_IN_FLASH; offs++) {
        flash_writer_write_float(pr->fw, offs, __float_reg[offs - MAX_INTS_IN_FLASH]);
    }
    pr->ready = false;
}

void preference_writer_load(PreferenceWriter pr) {
    int offs;
    for (offs = 0; offs < MAX_INTS_IN_FLASH; offs++) {
        __int_reg[offs] = flash_read_int(pr.fw, offs);
    }
    for(; offs < MAX_INTS_IN_FLASH+MAX_FLOATS_IN_FLASH; offs++) {
        __float_reg[offs - MAX_INTS_IN_FLASH] = flash_read_float(pr.fw, offs);
    }
}

void preference_writer_close(PreferenceWriter *pr) {
    pr->ready = false;
    flash_writer_close(&pr->fw);
}


