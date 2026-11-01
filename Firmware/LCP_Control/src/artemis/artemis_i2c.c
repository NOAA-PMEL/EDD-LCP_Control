/// @file artemis_i2c.c

#include "artemis_debug.h"
#include "artemis_i2c.h"
#include <string.h>

void artemis_i2c_send(artemis_i2c_t *i2c, bool stop, artemis_stream_t *txstream)
{
    am_hal_iom_transfer_t transfer = {0};
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    /* 32-bit aligned buffer required by Apollo3 IOM DMA hardware */
    uint32_t aligned_tx_buf[36]; 

    if (txstream == NULL || txstream->written == 0)
    {
        return;
    }

    uint32_t bytes_to_send = txstream->written;
    if (bytes_to_send > sizeof(aligned_tx_buf))
    {
        bytes_to_send = sizeof(aligned_tx_buf);
    }

    memcpy(aligned_tx_buf, txstream->buffer, bytes_to_send);

    transfer.uPeerInfo.ui32I2CDevAddr = i2c->address;
    transfer.bContinue = !stop;
    transfer.pui32TxBuffer = aligned_tx_buf;
    transfer.pui32RxBuffer = NULL;
    transfer.ui32NumBytes = bytes_to_send;
    transfer.eDirection = AM_HAL_IOM_TX;
    transfer.ui8Priority = 1;

    status = am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        ARTEMIS_DEBUG_PRINTF("I2C :: SEND ERROR Status=0x%X (%d)\n", status, status);
    }

    txstream->read = txstream->written;
}

void artemis_i2c_receive(artemis_i2c_t *i2c, bool stop, artemis_stream_t *rxstream, uint32_t rxnumber)
{
    am_hal_iom_transfer_t transfer = {0};
    uint32_t status = AM_HAL_STATUS_SUCCESS;
    /* 32-bit aligned buffer required by Apollo3 IOM DMA hardware */
    uint32_t aligned_rx_buf[36]; 

    if (rxstream == NULL || rxnumber == 0)
    {
        return;
    }

    uint32_t bytes_to_read = rxnumber;
    if (bytes_to_read > sizeof(aligned_rx_buf))
    {
        bytes_to_read = sizeof(aligned_rx_buf);
    }

    transfer.uPeerInfo.ui32I2CDevAddr = i2c->address;
    transfer.bContinue = !stop;
    transfer.pui32TxBuffer = NULL;
    transfer.pui32RxBuffer = aligned_rx_buf;
    transfer.ui32NumBytes = bytes_to_read;
    transfer.eDirection = AM_HAL_IOM_RX;
    transfer.ui8Priority = 1;

    status = am_hal_iom_blocking_transfer(i2c->iom.handle, &transfer);
    if (status != AM_HAL_STATUS_SUCCESS)
    {
        ARTEMIS_DEBUG_PRINTF("I2C :: RECEIVE ERROR Status=0x%X (%d)\n", status, status);
    }
    else
    {
        memcpy(rxstream->buffer, aligned_rx_buf, bytes_to_read);
    }

    rxstream->written = bytes_to_read;
}