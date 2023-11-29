#include "i2c_interrupt.h"

#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "pico/timeout_helper.h"
#include <climits>

#define NUM_I2C_PERIPHERALS 2
TaskHandle_t* i2c_task_handles[NUM_I2C_PERIPHERALS] = {nullptr,nullptr};

void _handler_i2c0() {
	// Get interrupt status
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	irq_set_enabled(I2C0_IRQ, false);

	uint32_t status = i2c0->hw->intr_stat;

	//i2c0->hw->clr_rd_req;
	if(i2c_task_handles[0] != nullptr){
		//xTaskNotify( *i2c_task_handles[0], 0, eNoAction); //Even though this
		// is not recommended it takes about 5us less longer for one 5byte i2c transfer
		xTaskNotifyFromISR( *i2c_task_handles[0], 0, eNoAction, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

void _handler_i2c1() {
	// Get interrupt status
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	irq_set_enabled(I2C1_IRQ, false);
	//uint32_t status = i2c1->hw->intr_stat;
	//i2c1->hw->clr_rd_req;

	if(i2c_task_handles[1] != nullptr){
		//xTaskNotify( *i2c_task_handles[1], 0, eNoAction);
		xTaskNotifyFromISR( *i2c_task_handles[1], 0, eNoAction, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
	}
}

I2C_Status I2C_Interrupt_Master::start_transaction(){
	return I2C_OK;
}

I2C_Status I2C_Interrupt_Master::end_transaction(){
	return I2C_OK;
}

static inline void i2c_reset(i2c_inst_t *i2c) {
	invalid_params_if(I2C, i2c != i2c0 && i2c != i2c1);
	reset_block(i2c == i2c0 ? RESETS_RESET_I2C0_BITS : RESETS_RESET_I2C1_BITS);
}

static inline void i2c_unreset(i2c_inst_t *i2c) {
	invalid_params_if(I2C, i2c != i2c0 && i2c != i2c1);
	unreset_block_wait(i2c == i2c0 ? RESETS_RESET_I2C0_BITS : RESETS_RESET_I2C1_BITS);
}

// Addresses of the form 000 0xxx or 111 1xxx are reserved. No slave should
// have these addresses.
static inline bool i2c_reserved_addr(uint8_t addr) {
	return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

I2C_Interrupt_Master::I2C_Interrupt_Master(i2c_inst_t* i2c, uint baudrate):
    		i2c{i2c},
			//task_handle{i2c_task_handle},
			baudrate{baudrate}{
				uint hw_index = i2c_hw_index(i2c);
				//i2c_task_handles[hw_index] = i2c_task_handle;
			};

uint I2C_Interrupt_Master::init()
{
	i2c_reset(i2c);
	i2c_unreset(i2c);
	i2c->restart_on_next = false;

	i2c->hw->enable = 0;

	// Configure as a fast-mode master with RepStart support, 7-bit addresses
	i2c->hw->con =
			I2C_IC_CON_SPEED_VALUE_FAST << I2C_IC_CON_SPEED_LSB |
			I2C_IC_CON_MASTER_MODE_BITS |
			I2C_IC_CON_IC_SLAVE_DISABLE_BITS |
			I2C_IC_CON_IC_RESTART_EN_BITS |
			I2C_IC_CON_TX_EMPTY_CTRL_BITS;

	// Set FIFO watermarks to 1 to make things simpler. This is encoded by a register value of 0.
	i2c->hw->tx_tl = 0;
	i2c->hw->rx_tl = 0;

	// Always enable the DREQ signalling -- harmless if DMA isn't listening
	i2c->hw->dma_cr = I2C_IC_DMA_CR_TDMAE_BITS | I2C_IC_DMA_CR_RDMAE_BITS;

	// Re-sets i2c->hw->enable upon returning:
	uint ret_val = i2c_set_baudrate(i2c, baudrate);

	// Enable the I2C interrupts we want to process
	i2c->hw->intr_mask = (I2C_IC_INTR_MASK_M_RD_REQ_BITS | I2C_IC_INTR_MASK_M_RX_FULL_BITS |
			I2C_IC_INTR_MASK_M_TX_ABRT_BITS | I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_TX_OVER_BITS);

	uint hw_index = i2c_hw_index(i2c);

	if(hw_index == 0){
		// Set up the interrupt handler to service I2C interrupts
		irq_set_exclusive_handler(I2C0_IRQ, _handler_i2c0);

		// Enable I2C interrupt
		//irq_set_enabled(I2C0_IRQ, true);
	}else{
		// Set up the interrupt handler to service I2C interrupts
		irq_set_exclusive_handler(I2C1_IRQ, _handler_i2c1);

		// Enable I2C interrupt
		//irq_set_enabled(I2C1_IRQ, true);
	}

	return ret_val;
}


I2C_Status I2C_Interrupt_Master::write_n_then_read_m(uint8_t addr, const uint8_t *src, size_t src_len, uint8_t* dest, size_t dest_len, bool repeated_start, bool stop_at_end, uint32_t timeout_ms) {
	invalid_params_if(I2C, addr >= 0x80); // 7-bit addresses
	invalid_params_if(I2C, i2c_reserved_addr(addr));
	// Synopsys hw accepts start/stop flags alongside data items in the same
	// FIFO word, so no 0 byte transfers.
	invalid_params_if(I2C, src_len == 0);
	invalid_params_if(I2C, ((int)src_len) < 0);

	uint32_t ulNotifiedValue;

	i2c->hw->enable = 0;
	i2c->hw->tar = addr;
	i2c->hw->enable = 1;

	uint hw_index = i2c_hw_index(i2c);

	BaseType_t _wait_retval = pdFALSE;
	TaskHandle_t this_task = xTaskGetCurrentTaskHandle();
	i2c_task_handles[hw_index] = &this_task;

	bool abort = false;
	bool timeout = false;

	uint32_t abort_reason = 0;
	int byte_ctr;
	uint32_t status  = 0;

	TickType_t timeout_ticks = (timeout_ms == 0 ? portMAX_DELAY : timeout_ms);

	int ilen = (int)src_len;
	for (byte_ctr = ilen-1; byte_ctr >= 0; --byte_ctr) {
		bool last = byte_ctr == 0;
		bool first = byte_ctr == ilen - 1;

		i2c->hw->data_cmd =
				bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
				bool_to_bit(last && !repeated_start) << I2C_IC_DATA_CMD_STOP_LSB |
				src[byte_ctr];

		i2c->hw->intr_mask = (I2C_IC_INTR_MASK_M_TX_ABRT_BITS | I2C_IC_INTR_MASK_M_TX_EMPTY_BITS | I2C_IC_INTR_MASK_M_TX_OVER_BITS);

		if(hw_index == 0){
			irq_set_enabled(I2C0_IRQ, true);
		}else{
			irq_set_enabled(I2C1_IRQ, true);
		}

		xTaskNotifyStateClear(this_task); // Making sure that no prior notifications mess up the steps
		_wait_retval = xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
				ULONG_MAX, /* Reset the notification value to 0 on exit. */
				&ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
				timeout_ticks );

		if(_wait_retval == pdFALSE){ //This means that the operation timed out
			return I2C_Timed_Out;
		}

		status = i2c1->hw->intr_stat;

		if ((status & I2C_IC_INTR_MASK_M_TX_ABRT_BITS) || (status & I2C_IC_INTR_MASK_M_TX_OVER_BITS)){
			abort_reason = i2c->hw->tx_abrt_source;
			if (abort_reason) {
				// Note clearing the abort flag also clears the reason, and
				// this instance of flag is clear-on-read! Note also the
				// IC_CLR_TX_ABRT register always reads as 0.
				i2c->hw->clr_tx_abrt;
				abort = true;
			}
			return I2C_TX_Abort;
		}

		asm("nop;");
	}

	i2c->restart_on_next = true;

	ilen = (int)dest_len;
	for (byte_ctr = ilen-1; byte_ctr >= 0; --byte_ctr) {
		bool last = byte_ctr == 0;
		bool first = byte_ctr == ilen - 1;
		//while (!i2c_get_write_available(i2c))
		//	tight_loop_contents();

		i2c->hw->data_cmd =
				bool_to_bit(first && i2c->restart_on_next) << I2C_IC_DATA_CMD_RESTART_LSB |
				bool_to_bit(last && stop_at_end) << I2C_IC_DATA_CMD_STOP_LSB |
				I2C_IC_DATA_CMD_CMD_BITS; // -> 1 for read

		i2c->hw->intr_mask = I2C_IC_INTR_MASK_M_RX_FULL_BITS;

		if(hw_index == 0){
			irq_set_enabled(I2C0_IRQ, true);
		}else{
			irq_set_enabled(I2C1_IRQ, true);
		}

		xTaskNotifyStateClear(this_task); // Making sure that no prior notifications mess up the steps
		_wait_retval = xTaskNotifyWait(0x00,      /* Don't clear any notification bits on entry. */
				ULONG_MAX, /* Reset the notification value to 0 on exit. */
				&ulNotifiedValue, /* Notified value pass out in ulNotifiedValue. */
				timeout_ticks );

		if(_wait_retval == pdFALSE){ //This means that the operation timed out
			return I2C_Timed_Out;
		}

		asm("nop;");

		dest[byte_ctr] = (uint8_t) i2c->hw->data_cmd;
		//*dest++ = (uint8_t) i2c->hw->data_cmd;

	}

	i2c->restart_on_next = !stop_at_end;
	return I2C_OK;
}

// Post TX Section

// Wait until the transmission of the address/data from the internal
// shift register has completed. For this to function correctly, the
// TX_EMPTY_CTRL flag in IC_CON must be set. The TX_EMPTY_CTRL flag
// was set in i2c_init.
/*
do {
   if (timeout_check) {
	   timeout = timeout_check(ts);
	   abort |= timeout;
   }
   tight_loop_contents();
} while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_TX_EMPTY_BITS));

// If there was a timeout, don't attempt to do anything else.
if (!timeout) {
   abort_reason = i2c->hw->tx_abrt_source;
   if (abort_reason) {
	   // Note clearing the abort flag also clears the reason, and
	   // this instance of flag is clear-on-read! Note also the
	   // IC_CLR_TX_ABRT register always reads as 0.
	   i2c->hw->clr_tx_abrt;
	   abort = true;
   }

   if (abort || (last && !nostop)) {
	   // If the transaction was aborted or if it completed
	   // successfully wait until the STOP condition has occured.

	   // TODO Could there be an abort while waiting for the STOP
	   // condition here? If so, additional code would be needed here
	   // to take care of the abort.
	   do {
		   if (timeout_check) {
			   timeout = timeout_check(ts);
			   abort |= timeout;
		   }
		   tight_loop_contents();
	   } while (!timeout && !(i2c->hw->raw_intr_stat & I2C_IC_RAW_INTR_STAT_STOP_DET_BITS));

	   // If there was a timeout, don't attempt to do anything else.
	   if (!timeout) {
		   i2c->hw->clr_stop_det;
	   }
   }
}*/

			// Post RX Section
			// Note the hardware issues a STOP automatically on an abort condition.
			// Note also the hardware clears RX FIFO as well as TX on abort,
			// because we set hwparam IC_AVOID_RX_FIFO_FLUSH_ON_TX_ABRT to 0.
			/*
if (abort)
   break;
			 */

			/*
	do {
		abort_reason = i2c->hw->tx_abrt_source;
		abort = (bool) i2c->hw->clr_tx_abrt;
		if (timeout_check) {
			timeout = timeout_check(ts);
			abort |= timeout;
		}
	} while (!abort && !i2c_get_read_available(i2c));

	if (abort)
		break;

			 *dst++ = (uint8_t) i2c->hw->data_cmd; */

			// A lot of things could have just happened due to the ingenious and
			// creative design of I2C. Try to figure things out.
			/*
if (abort) {
    if (timeout)
        rval = PICO_ERROR_TIMEOUT;
    else if (!abort_reason || abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_7B_ADDR_NOACK_BITS) {
        // No reported errors - seems to happen if there is nothing connected to the bus.
        // Address byte not acknowledged
        rval = PICO_ERROR_GENERIC;
    } else if (abort_reason & I2C_IC_TX_ABRT_SOURCE_ABRT_TXDATA_NOACK_BITS) {
        // Address acknowledged, some data not acknowledged
        rval = byte_ctr;
    } else {
        //panic("Unknown abort from I2C instance @%08x: %08x\n", (uint32_t) i2c->hw, abort_reason);
        rval = PICO_ERROR_GENERIC;
    }
} else {
    rval = byte_ctr;
} */

			// nostop means we are now at the end of a *message* but not the end of a *transfer*
