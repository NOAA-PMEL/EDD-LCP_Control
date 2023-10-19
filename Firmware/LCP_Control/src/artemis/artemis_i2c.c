/// @file artemis_i2c.c

#include "artemis_debug.h"
#include "artemis_i2c.h"

void artemis_i2c_send(artemis_i2c_t *i2c, bool stop, artemis_stream_t *txstream)
{
	am_hal_iom_transfer_t transfer = {0};
	uint32_t status = AM_HAL_STATUS_SUCCESS;

	transfer.uPeerInfo.ui32I2CDevAddr = i2c->address;
	transfer.bContinue = !stop;
	transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
	transfer.ui32NumBytes = txstream->written;
	transfer.eDirection = AM_HAL_IOM_TX;
	transfer.ui8Priority = 1;

	ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer));
	//status = am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer);
	//if (status != AM_HAL_STATUS_SUCCESS){
	//	ARTEMIS_DEBUG_PRINTF("I2C ERROR \n");
	//}

	// update the number of bytes read from the txstream
	txstream->read = txstream->written;
}

void artemis_i2c_receive(artemis_i2c_t *i2c, bool stop, artemis_stream_t *rxstream, uint32_t rxnumber)
{
	am_hal_iom_transfer_t transfer = {0};
	uint32_t status = AM_HAL_STATUS_SUCCESS;

	transfer.uPeerInfo.ui32I2CDevAddr = i2c->address;
	transfer.bContinue = !stop;
	transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
	transfer.ui32NumBytes = rxnumber;
	transfer.eDirection = AM_HAL_IOM_RX;
	transfer.ui8Priority = 1;

	ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer));
	//status = am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer);
	//if (status != AM_HAL_STATUS_SUCCESS){
	//	ARTEMIS_DEBUG_PRINTF("I2C ERROR \n");
	//}

	// update the number of bytes written to the rxstream
	rxstream->written = rxnumber;
}
