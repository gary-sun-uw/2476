#include <msp430.h>
#include <stdint.h>
#include "BMI270_SensorAPI/bmi270.h"
#include "BMI270_SensorAPI/bmi2_defs.h"

static volatile uint8_t TXData;
static volatile uint8_t RXData;

void bmi2_delay_us(uint32_t period, void* intf_ptr) {
    uint32_t i = period;
    while (i) {
        __delay_cycles(1);
        i -= 1;
    }
}

BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // rx_data = reg_data;
    // rx_len = len;
    // rx_count = 0;
    // rw_state = RECEIVING_REGTX;

    // P3OUT &= ~BIT7;  // Set CSB low to begin transmission
    // //UCA1IV &= ~(USCI_SPI_UCRXIFG | USCI_SPI_UCTXIFG);  // Clear interrupt vector
    // UCA1IE |= UCTXIE;                         // Enable USCI_A1 RX and TX interrupt

    // UCA1TXBUF = 0x80 | reg_addr;  // Transmit the first byte: register address with MSB=1
    // __bis_SR_register(LPM0_bits | GIE);

    // // UCA1IE &= ~(UCRXIE | UCTXIE);  // Disable USCI_A1 RX and TX interrupts
    // P3OUT |= BIT7;  // Set CSB high to indicate end of transmission
    ///////
    P3OUT &= ~BIT7;

    // transmit register address
    TXData = 0x80 | reg_addr;
    UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0,enable interrupts
    __no_operation();                         // For debug,Remain in LPM0
    __delay_cycles(2000);                     // Delay before next transmission

    // // transmit dummy byte
    // UCA1IE |= UCTXIE;
    // __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0,enable interrupts
    // __no_operation();                         // For debug,Remain in LPM0
    // __delay_cycles(2000);                     // Delay before next transmission

    // transmit another byte for each byte we expect to recieve,
    // and then read everything into reg_data
    int i;
    for (i = 0; i < len; i += 1) {
        UCA1IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0,enable interrupts
        __no_operation();                         // For debug,Remain in LPM0
        __delay_cycles(2000);                     // Delay before next transmission

        reg_data[i] = RXData;
    }
    P3OUT |= BIT7;

    return 0;
}


BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr) {
    // tx_data = reg_data;
    // tx_len = len;
    // tx_count = 0;
    // rw_state = TRANSMITTING;


    // P3OUT &= ~BIT7;  // Set CSB low to begin transmission
    // //UCA1IV &= ~(USCI_SPI_UCRXIFG | USCI_SPI_UCTXIFG);
    // UCA1IE |= UCTXIE;                         // Enable USCI_A1 TX interrupt

    // UCA1TXBUF = reg_addr;  // Transmit the first byte: register address with MSB=0
    // __bis_SR_register(LPM0_bits | GIE);

    // UCA1IE &= ~UCTXIE;  // Disable USCI_A1 TX interrupt
    // P3OUT |= BIT7;  // Set CSB high to indicate end of transmission


    P3OUT &= ~BIT7;

    // transmit register address
    TXData = reg_addr;
    UCA1IE |= UCTXIE;
    __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0,enable interrupts
    __no_operation();                         // For debug,Remain in LPM0
    //__delay_cycles(2000);                     // Delay before next transmission   
  

    // transmit another byte for each byte we expect to recieve,
    // and then read everything into reg_data
    int i;
    for (i = 0; i < len; i += 1) {
        TXData = reg_data[i];
        UCA1IE |= UCTXIE;
        __bis_SR_register(LPM0_bits | GIE);       // Enter LPM0,enable interrupts
        __no_operation();                         // For debug,Remain in LPM0
        //__delay_cycles(2000);                     // Delay before next transmission
    }
    P3OUT |= BIT7;

    return 0;
}

void init_bmi_device(struct bmi2_dev* bmi) {
    // Store MCLK frequency in uHz for delay calculation
    // mclk_uhz = CS_getACLK() / 1000000;

    bmi->intf = BMI2_SPI_INTF;

    bmi->read = bmi2_spi_read;
    bmi->write = bmi2_spi_write;
    bmi->delay_us = bmi2_delay_us;

    // i am not using this, you can if you want
    bmi->intf_ptr = NULL;

    // Configure max read/write length (in bytes) ( Supported length depends on target machine)
    bmi->read_write_len = 46;

    // Assign to NULL to load the default config file.
    bmi->config_file_ptr = NULL;
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(UCA1IV,USCI_SPI_UCTXIFG))
  {
    case USCI_NONE: break;                    // Vector 0 - no interrupt
    case USCI_SPI_UCRXIFG:
           RXData = UCA1RXBUF;
           UCA1IFG &= ~UCRXIFG;
           __bic_SR_register_on_exit(LPM0_bits);// Wake up to setup next TX
           break;
    case USCI_SPI_UCTXIFG:
          UCA1TXBUF = TXData;                // Transmit characters
          UCA1IE &= ~UCTXIE;
          break;
    default: break;
    //     ////////
    // case USCI_NONE: break;                    // Vector 0 - no interrupt
    // case USCI_SPI_UCRXIFG:
    //     switch (rw_state) {
    //         case TRANSMITTING:
    //             // this shouldn't happen, but if it does it's probably ok to ignore it
    //             break;
    //         case RECEIVING:
    //             // receive the next byte in the sequence
    //             rx_data[rx_count] = UCA1RXBUF;
    //             UCA1IFG &= ~UCRXIFG;
    //             rx_count += 1;
    //             if (rx_count == rx_len) {
    //                 // we're done receiving data
    //                 rw_state = NONE;
    //                 UCA1IE &= ~UCTXIE;
    //                 __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
    //             }

    //             //__delay_cycles(2000);


    //             UCA1IE |= UCTXIE;
    //             break;
    //         case RECEIVING_REGTX:
    //             // data has ended up in the receive buffer while transmitting the register address,
    //             // so clear the ~~buffer~~ interrupt flag
    //             dummy = UCA1RXBUF;
    //             UCA1IFG &= ~UCRXIFG;

    //             UCA1IE |= UCTXIE;
    //         default:
    //             break;
    //     }
    //     break;

           
    // case USCI_SPI_UCTXIFG:
    //     switch (rw_state) {
    //         case TRANSMITTING:
    //             // transmit the next byte in the sequence
                
    //             UCA1TXBUF = tx_data[tx_count];                // Transmit characters
    //             tx_count += 1;
    //             if (tx_count == tx_len) {
    //                 // we're done transmitting data
    //                 rw_state = NONE;
    //                 UCA1IE &= ~UCTXIE;
    //                 __bic_SR_register_on_exit(LPM0_bits); // leave low power mode
    //             }
    //             break;
    //         case RECEIVING_REGTX:
    //             rw_state = RECEIVING;
    //         case RECEIVING:
    //             // for every byte we expect to receive, we need to transmit a dummy byte to get
    //             // the clock going for 8 cycles
    //             // the BMI270 will transfer data to us during those cycles, and then the UCRXIFG
    //             // interrupt will fire and we can read it
                
    //             UCA1TXBUF = 0;
    //             UCA1IE &= ~UCTXIE;
    //             break;
    //         default:
    //             break;
    //     }
    //     break;
    // default: break;
  }
}
