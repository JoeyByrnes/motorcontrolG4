/*
 * position_sensor.c
 *
 *  Created on: Jul 26, 2020
 *      Author: Ben
 */
#include <stdio.h>
#include <string.h>
#include "position_sensor.h"
#include "math_ops.h"
#include "hw_config.h"
#include "user_config.h"

#define AS4047D_CS1_Port GPIOB
#define AS4047D_CS1_Pin GPIO_PIN_11
#define ENC_CS_HIGH AS4047D_CS1_Port->BSRR=(uint32_t)AS4047D_CS1_Pin
#define ENC_CS_LOW AS4047D_CS1_Port->BRR=(uint32_t)AS4047D_CS1_Pin
#define READ_POS_REG 0xFFFF

void ps_warmup(EncoderStruct * encoder, int n){
	/* Hall position sensors noisy on startup.  Take a bunch of samples to clear this data */
	for(int i = 0; i<n; i++){
		encoder->spi_tx_word = 0x0000;
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
		HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 1, 100);
		while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
		HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
	}
}

uint16_t AS5047_Read_Pos()
{
	ENC_CS_LOW;
	while ((SPI2->SR & SPI_SR_BSY) != 0);
	SPI2->DR = READ_POS_REG;
	SPI2->CR1 |= SPI_CR1_SPE;
	while ((SPI2->SR & SPI_SR_RXNE) == 0);
	uint16_t result = SPI2->DR;
	while ((SPI2->SR & SPI_SR_TXE) == 0);
	while ((SPI2->SR & SPI_SR_BSY) != 0);
	SPI2->CR1 &= ~(SPI_CR1_SPE);
	ENC_CS_HIGH;
	return (result & 0x3FFF);
}

void ps_sample(EncoderStruct * encoder, float dt){
	/* updates EncoderStruct encoder with the latest sample
	 * after elapsed time dt */

	/* Shift around previous samples */
	encoder->old_angle = encoder->angle_singleturn;
	for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->angle_multiturn[i] = encoder->angle_multiturn[i-1];}
	//for(int i = N_POS_SAMPLES-1; i>0; i--){encoder->count_buff[i] = encoder->count_buff[i-1];}
	//memmove(&encoder->angle_multiturn[1], &encoder->angle_multiturn[0], (N_POS_SAMPLES-1)*sizeof(float)); // this is much slower for some reason

	/* SPI read/write */
//	encoder->spi_tx_word = ENC_READ_WORD;
//	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_RESET ); 	// CS low
//	HAL_SPI_TransmitReceive(&ENC_SPI, (uint8_t*)encoder->spi_tx_buff, (uint8_t *)encoder->spi_rx_buff, 2, 100);
//	while( ENC_SPI.State == HAL_SPI_STATE_BUSY );  					// wait for transmission complete
//	HAL_GPIO_WritePin(ENC_CS, GPIO_PIN_SET ); 	// CS high
//	encoder->raw = encoder ->spi_rx_word;
	encoder->raw = AS5047_Read_Pos(); // JB

	/* Linearization */
	int off_1 = encoder->offset_lut[(encoder->raw)>>9];				// lookup table lower entry
	int off_2 = encoder->offset_lut[((encoder->raw>>9)+1)%128];		// lookup table higher entry
	int off_interp = off_1 + ((off_2 - off_1)*(encoder->raw - ((encoder->raw>>9)<<9))>>9);     // Interpolate between lookup table entries
	encoder->count = encoder->raw + off_interp;


	/* Real angles in radians */
	encoder->angle_singleturn = ((float)(encoder->count-M_ZERO))/((float)ENC_CPR);
	int int_angle = encoder->angle_singleturn;
	encoder->angle_singleturn = TWO_PI_F*(encoder->angle_singleturn - (float)int_angle);
	//encoder->angle_singleturn = TWO_PI_F*fmodf(((float)(encoder->count-M_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->angle_singleturn = encoder->angle_singleturn<0 ? encoder->angle_singleturn + TWO_PI_F : encoder->angle_singleturn;

	encoder->elec_angle = (encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR);
	int_angle = (int)encoder->elec_angle;
	encoder->elec_angle = TWO_PI_F*(encoder->elec_angle - (float)int_angle);
	//encoder->elec_angle = TWO_PI_F*fmodf((encoder->ppairs*(float)(encoder->count-E_ZERO))/((float)ENC_CPR), 1.0f);
	encoder->elec_angle = encoder->elec_angle<0 ? encoder->elec_angle + TWO_PI_F : encoder->elec_angle;	// Add 2*pi to negative numbers
	/* Rollover */
	int rollover = 0;
	float angle_diff = encoder->angle_singleturn - encoder->old_angle;
	if(angle_diff > PI_F){rollover = -1;}
	else if(angle_diff < -PI_F){rollover = 1;}
	encoder->turns += rollover;
	if(!encoder->first_sample){
		encoder->turns = 0;
		encoder->first_sample = 1;
	}



	/* Multi-turn position */
	encoder->angle_multiturn[0] = encoder->angle_singleturn + TWO_PI_F*(float)encoder->turns;

	/* Velocity */
	/*
	// Attempt at a moving least squares.  Wasn't any better
		float m = (float)N_POS_SAMPLES;
		float w = 1.0f/m;
		float q = 12.0f/(m*m*m - m);
		float c1 = 0.0f;
		float ibar = (m - 1.0f)/2.0f;
		for(int i = 0; i<N_POS_SAMPLES; i++){
			c1 += encoder->angle_multiturn[i]*q*(i - ibar);
		}
		encoder->vel2 = -c1/dt;
*/
	//encoder->velocity = vel2

//	float old_pos_sum = 0;
//	float new_pos_sum = 0;
//
//	for(int i = 0; i<10; i++){
//		new_pos_sum += encoder->angle_multiturn[i];
//	}
//	for(int i = 10; i<20; i++){
//		old_pos_sum += encoder->angle_multiturn[i];
//	}
//
//	float old_pos_avg = old_pos_sum/10.0;
//	float new_pos_avg = new_pos_sum/10.0;
//
//	float vel = (new_pos_avg - old_pos_avg)/(dt*9.0);



	encoder->velocity = (encoder->angle_multiturn[0] - encoder->angle_multiturn[N_POS_SAMPLES-1])/(dt*(float)(N_POS_SAMPLES-1));
	encoder->elec_velocity = encoder->ppairs*encoder->velocity;

}

void ps_print(EncoderStruct * encoder, int dt_ms){
	printf("Raw: %d", encoder->raw);
	printf("   Linearized Count: %d", encoder->count);
	printf("   Single Turn: %f", encoder->angle_singleturn);
	printf("   Multiturn: %f", encoder->angle_multiturn[0]);
	printf("   Electrical: %f", encoder->elec_angle);
	printf("   Turns:  %d\r\n", encoder->turns);
	//HAL_Delay(dt_ms);
}
