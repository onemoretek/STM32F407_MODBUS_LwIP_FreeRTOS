/*! ----------------------------------------------------------------------------
 * @file	deca_spi.c
 * @brief	SPI access functions
 *
 * @attention
 *
 * Copyright 2013 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include <string.h>

#include "deca_spi.h"
#include "deca_sleep.h"
#include "deca_device_api.h"
#include "port.h"
#include "main.h"

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
int writetospi_serial(
					uint16 headerLength,
			   	const uint8 *headerBuffer,
					uint32 bodylength,
					const uint8 *bodyBuffer
				  );

int readfromspi_serial(
					 uint16	headerLength,
			     const uint8 *headerBuffer,
					 uint32 readLength,
					 uint8 *readBuffer
					 );
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
	// done by port.c, default SPI used is SPI1

	return 0;

} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
//	while (port_SPIx_busy_sending()); //wait for tx buffer to empty

//	port_SPIx_disable();

	return 0;

} // end closespi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi_serial()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
#pragma GCC optimize ("O3")
int writetospi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       bodyLength,
    const uint8 *bodyBuffer
)
{
//    decaIrqStatus_t  stat ;
//    stat = decamutexon() ;

	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(&hspi2, (uint8_t *)&headerBuffer[0], headerLength, 10);	/* Send header in polling mode */
    HAL_SPI_Transmit(&hspi2, (uint8_t *)&bodyBuffer[0], bodyLength, 10);		/* Send data in polling mode */

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */


//    decamutexoff(stat) ;

    return 0;
} // end writetospi_serial()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi_serial()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
#pragma GCC optimize ("O3")
int readfromspi_serial
(
    uint16       headerLength,
    const uint8 *headerBuffer,
    uint32       readLength,
    uint8       *readBuffer
)
{
	uint8_t spi_TmpBuffer[BUFFLEN];
	assert_param(headerLength+readLength < BUFFLEN );
	
//    decaIrqStatus_t  stat ;
//    stat = decamutexon() ;

	/* Blocking: Check whether previous transfer has been finished */
	while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)headerBuffer, spi_TmpBuffer, (uint16_t)(headerLength+readLength), 10);

	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */

	memcpy((uint8_t*)readBuffer , (uint8_t*)&spi_TmpBuffer[headerLength], readLength);

//	decamutexoff(stat);

    return 0;
} // end readfromspi_serial()
