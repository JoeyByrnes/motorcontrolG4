#ifndef HW_CONFIG_H
#define HW_CONFIG_H

extern FDCAN_HandleTypeDef hfdcan1;


/* Timer and PWM */
#define TIM_PWM			htim2				// PWM/ISR timer handle
#define TIM_CH_U		TIM_CHANNEL_3		// Terminal U timer channel
#define TIM_CH_V		TIM_CHANNEL_2		// Terminal V timer channel
#define TIM_CH_W		TIM_CHANNEL_1		// Terminal W timer channel
#define INVERT_DTC		1					// PWM inverting (1) or non-inverting (0)

/* ISRs */
#define PWM_ISR			TIM2_IRQn	// PWM Timer ISR
#define CAN_ISR			FDCAN1_IT0_IRQn		// CAN Receive ISR

/* ADC */

#define ADC_CH_MAIN		hadc1				// ADC channel handle which drives simultaneous mode
#define ADC_CH_IA		hadc1				// Phase A current sense ADC channel handle.  0 = unused
#define ADC_CH_IB		hadc2				// Phase B current sense ADC channel handle.  0 = unused
#define ADC_CH_IC		hadc3				// Phase C current sense ADC channel handle.  0 = unused
#define ADC_CH_VBUS		hadc4				// Bus voltage ADC channel handle.  0 = unused

/* DRV Gate drive */
#define ENABLE_PIN 		GPIOA,GPIO_PIN_3  // Enable gate drive pin.
#define DRV_SPI			hspi1				// DRV SPI handle
#define DRV_CS			GPIOC,GPIO_PIN_4	// DRV CS pin

/* SPI encoder */
#define ENC_SPI			hspi2				// Encoder SPI handle
#define ENC_CS			GPIOB,GPIO_PIN_11	// Encoder SPI CS pin
#define ENC_CPR			16384				// Encoder counts per revolution
#define INV_CPR			1.0f/ENC_CPR
#define ENC_READ_WORD	0x3FFF				// Encoder read command

/* Misc. GPIO */
#define LED         	GPIOC,GPIO_PIN_5	// LED Pin

/* CAN */
#define CAN_H			hfdcan1				// CAN handle

/* Other hardware-related constants */
#define I_SCALE 			0.0201416f  // Amps per A/D Count at 40X amplifier gain
#define V_SCALE 			1.592*0.0128906f    // Bus volts per A/D Count !!!!!!!should be 0.0179475f //JB
#define DTC_MAX 			0.94f          	// Max duty cycle
#define DTC_MIN 			0.0f          	// Min duty cycle
#define DTC_COMP 			0.000f          // deadtime compensation (100 ns / 25 us)
#define DT					.000025f		// Loop period // change back to .000025f later //JB
#define EN_ENC_LINEARIZATION 1				// Enable/disable encoder linearization


/* Current controller */
#define L_D .000003f				// D axis inductance
#define L_Q .000003f				// Q axis inductance
#define K_D .05f                    // Loop gain,  Volts/Amp
#define K_Q .05f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.045f                // PI zero, in radians per sample
#define KI_Q 0.045f                // PI zero, in radians per sample
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation
#define CURRENT_FILT_ALPHA	.1f	// 1st order d/q current filter (not used in control)
#define VBUS_FILT_ALPHA		.1f		// 1st order bus voltage filter

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples


#endif
