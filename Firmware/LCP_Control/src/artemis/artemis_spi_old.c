//
/// @file artemis_spi.c
///

#include "artemis_debug.h"
#include "artemis_spi.h"

#include "am_hal_gpio.h"




static void module_set_chipselect(artemis_spi_bb_t *spi);
static void module_clear_chipselect(artemis_spi_bb_t *spi);
static void module_toggle_clk_on_off(artemis_spi_bb_t *spi);
static bool module_spi_read_bit(artemis_spi_bb_t *spi);
static bool module_spi_write_bit(artemis_spi_bb_t *spi, bool value);

///
///
///
void artemis_spi_send(artemis_spi_t *spi, bool stop, artemis_stream_t *txstream)
{
    am_hal_iom_transfer_t transfer = {0};

    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
    transfer.bContinue = !stop;
    transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
    transfer.ui32NumBytes = txstream->written;
    transfer.eDirection = AM_HAL_IOM_TX;
    transfer.ui8Priority = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));

    // update the number of bytes read from the txstream
    txstream->read = txstream->written;
}

///
///
///

//void artemis_spi_receive(artemis_spi_t *spi, bool stop, uint8_t cmd, artemis_stream_t *rxstream, uint32_t rxnumber)
//{  
//  
//  uint32_t data[64];
//    am_hal_iom_transfer_t       Transaction;
//
//    Transaction.ui32InstrLen    = 1;
//    Transaction.ui32Instr       = cmd;
//    Transaction.eDirection      = AM_HAL_IOM_RX;
//    Transaction.ui32NumBytes    = rxnumber;
//    Transaction.pui32RxBuffer   = data;
//    Transaction.bContinue       = false;
//    Transaction.ui8RepeatCount  = 0;
//    Transaction.ui32PauseCondition = 0;
//    Transaction.ui32StatusSetClr = 0;
//    Transaction.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
//
//
//    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &Transaction));
//    
//    artemis_stream_write(rxstream, (uint8_t*)data, rxnumber);
//}


void artemis_spi_receive(artemis_spi_t *spi, bool stop, artemis_stream_t *rxstream, uint32_t rxnumber)
{
    am_hal_iom_transfer_t transfer = {0};

    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
    transfer.bContinue = !stop;
    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
    transfer.ui32NumBytes = rxnumber;
    transfer.eDirection = AM_HAL_IOM_RX;
    transfer.ui8Priority = 0;
    transfer.ui32InstrLen = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));

    // update the number of bytes written to the rxstream
    rxstream->written = rxnumber;
}

void artemis_spi_rx(artemis_spi_t *spi, bool stop, artemis_stream_t *rxstream, uint32_t rxnumber)
{
  
    am_hal_iom_transfer_t transfer = {0};

    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
    transfer.bContinue = !stop;
    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
    transfer.ui32NumBytes = rxnumber;
    transfer.eDirection = AM_HAL_IOM_RX;
    transfer.ui32Instr = 0x01;
    transfer.ui32InstrLen = 1;
    transfer.ui8RepeatCount = 0;
    transfer.ui8Priority = 1;
    transfer.ui32StatusSetClr = 0;

    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));

    // update the number of bytes written to the rxstream
    rxstream->written = rxnumber;
}

///
///
///
//void artemis_spi_transfer(artemis_spi_t *spi, bool stop, artemis_stream_t *txstream, artemis_stream_t *rxstream)
//{
//    am_hal_iom_transfer_t transfer = {0};
//
//    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
//    transfer.bContinue = true;
//    transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
//    transfer.pui32RxBuffer = NULL;
//    transfer.ui32NumBytes = 1;
//    transfer.eDirection = AM_HAL_IOM_TX;
//    transfer.ui8Priority = 1;
//    
////    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));
////    
//    transfer.ui32NumBytes = txstream->written-1;
//    transfer.bContinue = !stop;
//    transfer.eDirection = AM_HAL_IOM_RX;                        
//    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
//    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));
////    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_spi_blocking_fullduplex(spi->iom.handle, &transfer));
//
//    // update the number of bytes read from the txstream
//    txstream->read = txstream->written;
//
//    // update the number of bytes written to the rxstream
//    rxstream->written = txstream->written;
//}
//
//void artemis_spi_transfer(artemis_spi_t *spi, bool stop, artemis_stream_t *txstream, artemis_stream_t *rxstream)
//{
//    am_hal_iom_transfer_t transfer = {0};
//
//    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
//    transfer.bContinue = !stop;
//    transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
//    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
//    transfer.ui32NumBytes = txstream->written;
//    transfer.eDirection = AM_HAL_IOM_FULLDUPLEX;
//    transfer.ui8Priority = 1;
//    transfer.ui32Instr = 0;
//    transfer.ui32InstrLen = 0;
//
//    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_spi_blocking_fullduplex(spi->iom.handle, &transfer));
//
//    // update the number of bytes read from the txstream
//    txstream->read = txstream->written;
//
//    // update the number of bytes written to the rxstream
//    rxstream->written = txstream->written;
//}

void artemis_spi_transfer(artemis_spi_t *spi, bool stop, artemis_stream_t *txstream, artemis_stream_t *rxstream)
{
    am_hal_iom_transfer_t transfer = {0};

    transfer.uPeerInfo.ui32SpiChipSelect = spi->chipselect;
    transfer.bContinue = true;
//    transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
//    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
    transfer.ui32NumBytes = 0;
    transfer.eDirection = AM_HAL_IOM_TX;
    transfer.ui8Priority = 1;
    transfer.ui32Instr = 0;
    transfer.ui32InstrLen = 0;
    
    artemis_stream_get(txstream, (uint8_t*)&transfer.ui32Instr);
    transfer.ui32InstrLen = 1;
    
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));
    
    
    transfer.ui32Instr = 0;
    transfer.ui32InstrLen = 0;
    transfer.eDirection = AM_HAL_IOM_RX;
    transfer.bContinue = !stop;
    transfer.ui32NumBytes = txstream->written;
    transfer.pui32RxBuffer = (uint32_t *)rxstream->buffer;
    transfer.pui32TxBuffer = (uint32_t *)txstream->buffer;
    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_blocking_transfer(spi->iom.handle, &transfer));
//    ARTEMIS_DEBUG_HALSTATUS(am_hal_iom_spi_blocking_fullduplex(spi->iom.handle, &transfer));

    // update the number of bytes read from the txstream
    txstream->read = txstream->written;

    // update the number of bytes written to the rxstream
    rxstream->written = txstream->written;
}





void artemis_spi_bb_send(artemis_spi_bb_t *spi, bool stop, artemis_stream_t *txstream)
{
    while(txstream->length)
    {
        uint8_t write_byte = 0;
        for(uint8_t i=0;i<8;i++)
        {
            module_spi_write_bit(spi, (write_byte & (1 << i)) );
            module_toggle_clk_on_off(spi);
        }
    }
}

void artemis_spi_bb_receive(artemis_spi_bb_t *spi, bool stop, artemis_stream_t *rxstream, uint32_t ui32Length)
{
    while(ui32Length-- > 0)
    {
        uint8_t read_byte = 0;
        for(uint8_t i=0;i<8;i++)
        {
            module_spi_write_bit(spi, 0);
            module_toggle_clk_on_off(spi);
            read_byte |= (module_spi_read_bit(spi) << i);
        }
        artemis_stream_put(rxstream, read_byte);
    }
}

void artemis_spi_bb_transfer(artemis_spi_bb_t *spi, bool stop, artemis_stream_t *txstream, artemis_stream_t *rxstream)
{
    uint32_t length = txstream->length;
    uint8_t data_byte = 0;
    module_set_chipselect(spi);
    for(uint32_t i=0; i<length; i++)
    {
        artemis_stream_get(txstream, &data_byte);
        uint8_t read_byte;
        
        for(uint8_t j=0; j<8; j++)
        {
            bool bit_to_send = (data_byte & (1u < j)) > 0;
            module_spi_write_bit(spi, bit_to_send);
            module_toggle_clk_on_off(spi);
            read_byte |= (module_spi_read_bit(spi) << j);
        }
        artemis_stream_put(rxstream, read_byte);
    }

    if(!stop)
    {
        module_clear_chipselect(spi);
    }

}

static void module_set_chipselect(artemis_spi_bb_t *spi)
{
    am_hal_gpio_state_write(spi->MOSI, AM_HAL_GPIO_OUTPUT_CLEAR);
}

static void module_clear_chipselect(artemis_spi_bb_t *spi)
{
    am_hal_gpio_state_write(spi->MOSI, AM_HAL_GPIO_OUTPUT_SET);
}

static void module_toggle_clk_on_off(artemis_spi_bb_t *spi)
{
    am_hal_gpio_state_write(spi->SCLK, AM_HAL_GPIO_OUTPUT_CLEAR);
    am_hal_systeick_delay_us(2);
    am_hal_gpio_state_write(spi->SCLK, AM_HAL_GPIO_OUTPUT_SET);
//    am_hal_gpio_interrupt_clear(spi->SCLK);
//    am_hal_systick_delay_us(10);
//    am_hal_gpio_interrupt_set(sclk);
}


static bool module_spi_read_bit(artemis_spi_bb_t *spi)
{
    uint32_t data;
    am_hal_gpio_state_read(spi->MISO, AM_HAL_GPIO_INPUT_READ, &data);
    return (data > 0);
}

static bool module_spi_write_bit(artemis_spi_bb_t *spi, bool value)
{
    if( value == true)
    {
        am_hal_gpio_state_write(spi->MOSI, AM_HAL_GPIO_OUTPUT_SET);
    } else {
        am_hal_gpio_state_write(spi->MOSI, AM_HAL_GPIO_OUTPUT_CLEAR);
    }

}