#include "stm32g4xx.h"
#include "flash_writer.h"

#define SIZE_OFFSET 16

static void getFLASHSectorForEEPROM(uint32_t address, uint32_t *bank, uint32_t *sector)
{
#if defined(FLASH_BANK2_BASE)
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK2_BASE) {
        *bank = FLASH_BANK_1;
    } else if (address >= FLASH_BANK2_BASE && address < FLASH_BANK2_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_2;
        address -= FLASH_BANK_SIZE;
    }
#else
    if (address >= FLASH_BANK1_BASE && address < FLASH_BANK1_BASE + FLASH_BANK_SIZE) {
        *bank = FLASH_BANK_1;
    }
#endif
    else {
        // Not good

    }

    address -= FLASH_BANK1_BASE;
    *sector = address / FLASH_PAGE_SIZE;
}

void g4_flash_erase(uint8_t page){
    uint32_t page_err = 0;
    FLASH_EraseInitTypeDef erase_conf;
    erase_conf.TypeErase = FLASH_TYPEERASE_MASSERASE;
    erase_conf.Banks = FLASH_BANK_2;

    if (HAL_FLASHEx_Erase(&erase_conf, &page_err) != HAL_OK)
    {
    	printf("\r\nErasing Flash Failed, Please Reboot Manually.\r\n");

    }
    else{
    	printf("\r\nFlash Erase Successful\r\n");
    }
  }

static void FLASH_Program_Word(uint32_t Address, uint32_t Data) // JB
{
  /* Check the parameters */
  assert_param(IS_FLASH_PROGRAM_ADDRESS(Address));

  /* Set PG bit */
  SET_BIT(FLASH->CR, FLASH_CR_PG);

  /* Program word */
  *(uint32_t *)Address = (uint32_t)Data;


}

// STM32G4 FUNCTIONS ABOVE =======================================================

// STANDARD FUNCTIONS BELOW ======================================================

void flash_writer_init(FlashWriter *fw, uint32_t sector) {
	if(sector>7) sector = 7;
	fw->sector = sector;
	fw->base = PW_BASE;
	fw->ready = false;
}

bool flash_writer_ready(FlashWriter fw) {
    return fw.ready;
}


void flash_writer_open(FlashWriter * fw) {
	HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    g4_flash_erase(0);
    fw->ready = true;
}

void flash_writer_write_int(FlashWriter fw, uint32_t index, int x) {
    union UN {int a; uint32_t b;};
    union UN un;
    un.a = x;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PW_BASE + SIZE_OFFSET * index, un.b); //JB
}

void flash_writer_write_uint(FlashWriter fw, uint32_t index, unsigned int x) {
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PW_BASE + SIZE_OFFSET * index, x); //JB
}

void flash_writer_write_float(FlashWriter fw, uint32_t index, float x) {
    union UN {float a; uint32_t b;};
    union UN un;
    un.a = x;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, PW_BASE + SIZE_OFFSET * index, un.b); //JB
}

void flash_writer_close(FlashWriter * fw) {
	HAL_FLASH_Lock(); //JB
    fw->ready = false;
}

int flash_read_int(FlashWriter fw, uint32_t index) {
    return *(int*) (PW_BASE + SIZE_OFFSET * index);
}

uint32_t flash_read_uint(FlashWriter fw, uint32_t index) {
    return *(uint32_t*) (PW_BASE + SIZE_OFFSET * index);
}

float flash_read_float(FlashWriter fw, uint32_t index) {
    return *(float*) (PW_BASE + SIZE_OFFSET * index);
}


