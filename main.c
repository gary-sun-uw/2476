/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#include "driverlib.h"
#include "bmi270_spi.h"
#include "BMI270_SensorAPI/bmi270.h"
#include "interrupt.h"
#include "classification/classify.h"
#include "commons/data_formats.h"
#include "commons/constants.h"

//******************************************************************************
//!
//!   Empty Project that includes driverlib
//!
//******************************************************************************

unsigned int int1_status = 0;
unsigned int int2_status = 0;
unsigned int int1_status_result = 0;
unsigned int int2_status_result = 0;
unsigned int interval_setting_mode = 0;
unsigned int buzz_en = 0;
unsigned int buzz_x = 2;
unsigned int counter = 0;
unsigned int timer_in_seconds = 0;
unsigned int alarm_en = 0;
unsigned int alarm_thres = 10;

void init_spi();
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

void init_GPIO() {
    // buzzer
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);

    // P4.0 interrupt
    // BMI INT2
    GPIO_selectInterruptEdge(GPIO_PORT_P4, GPIO_PIN0, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);

    // P3.1 interrupt
    // BMI INT1
    GPIO_selectInterruptEdge(GPIO_PORT_P3, GPIO_PIN1, GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_setAsInputPin(GPIO_PORT_P3, GPIO_PIN1);
    GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN1);
    GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN1);
}

void init_timerA1() {

    //Start timer in continuous mode sourced by ACLK
    Timer_A_clearTimerInterrupt(TIMER_A1_BASE);

    Timer_A_initUpModeParam param = {0};
    param.clockSource = TIMER_A_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_A_DO_CLEAR;
    param.captureCompareInterruptEnable_CCR0_CCIE =  TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE;
    param.startTimer = true;
    param.timerPeriod = 6000;
    Timer_A_initUpMode(TIMER_A1_BASE, &param);

    //Timer_A_StartCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
}

void init_timerB() {
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);

    Timer_B_initUpModeParam param = {0};
    param.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod = 32768;
    param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_ENABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE;
    param.timerClear = TIMER_B_DO_CLEAR;
    param.startTimer = false;
    Timer_B_initUpMode(TIMER_B0_BASE, &param);

//    Timer_B_startCounter(TIMER_B0_BASE, TIMER_B_UP_MODE);
//    Timer_B_stop(TIMER_B0_BASE);

}

void main (void)
{
    //Stop WDT
    WDT_A_hold(WDT_A_BASE);

    PMM_unlockLPM5();
    init_spi();
    init_GPIO();
    init_timerA1();
    init_timerB();

    static struct bmi2_dev bmi;
    init_bmi_device(&bmi);

    #define SENS_NUM 4
    uint8_t sens_list[SENS_NUM] = { BMI2_ACCEL, BMI2_GYRO, BMI2_WRIST_GESTURE, BMI2_NO_MOTION };

    struct bmi2_feat_sensor_data sens_data = { 0 };
    sens_data.type = BMI2_WRIST_GESTURE;

    struct bmi2_sens_data sensor_data_temp = {0};

    alarm_thres = default_interval_setting.interval_mins;

    bmi.aps_status = BMI2_ENABLE;

    int8_t rslt = bmi270_init(&bmi);
    if (rslt == BMI2_OK){
        set_accel_gyro_config(&bmi);
        setup_interrupt_pin(&bmi);
        setup_features(&bmi);
        bmi270_sensor_enable(sens_list, SENS_NUM, &bmi);
        bmi270_map_feat_int(feat_int_map, NUM_FEAT, &bmi);

        do {
            //Renable interrupts
            GPIO_clearInterrupt(GPIO_PORT_P3, GPIO_PIN1);
            GPIO_enableInterrupt(GPIO_PORT_P3, GPIO_PIN1);

            GPIO_clearInterrupt(GPIO_PORT_P4, GPIO_PIN0);
            GPIO_enableInterrupt(GPIO_PORT_P4, GPIO_PIN0);

            if(interval_setting_mode == 1){
                Timer_B_startCounter(TIMER_B0_BASE, TIMER_B_UP_MODE);
                if(alarm_en == 1){
                    alarm_en = 0;
                    timer_in_seconds = 0;
                }
            }

            __bis_SR_register(LPM3_bits + GIE);

            if(interval_setting_mode == 1){
                int indx = 0;
                Frame f[5];
                while (indx < 5)
                {
                    rslt = bmi2_get_sensor_data(&sensor_data_temp, &bmi);
                    if ((rslt == BMI2_OK) && (sensor_data_temp.status & BMI2_DRDY_ACC) &&
                        (sensor_data_temp.status & BMI2_DRDY_GYR))
                    {
                        f[indx] = (struct Frame){
                            .acc = (struct Vec3d){
                                .x = sensor_data_temp.acc.x,
                                .y = sensor_data_temp.acc.y,
                                .z = sensor_data_temp.acc.z
                            },
                            .gyr = (struct Vec3d){
                                .x = sensor_data_temp.gyr.x,
                                .y = sensor_data_temp.gyr.y,
                                .z = sensor_data_temp.gyr.z
                            }
                        };

                        indx++;
                    }
                }

                IntervalSetting result = classify_interval_setting(f, 5, interval_vector_min_similarity);
                if(result.interval_mins != -1){
                    alarm_thres = result.interval_mins;

                    switch (result.interval_mins) {
                        case 5:
                            buzz_x = 2;
                            break;
                        case 10:
                            buzz_x = 4;
                            break;
                        case 20:
                            buzz_x = 6;
                            break;
                        case 40:
                            buzz_x = 8;
                            break;
                    }

                    buzz_en = 1;
                    interval_setting_mode = 0;
                    Timer_B_stop(TIMER_B0_BASE);
                }
            } else {
                if(alarm_en == 1){
                    Timer_B_stop(TIMER_B0_BASE);
                    timer_in_seconds = 0;
                    buzz_x = 2;
                    buzz_en = 1;
                }

                if(int2_status == 1){
                    int2_status = 0;
                    bmi2_get_int_status(&int2_status_result, &bmi);

                    if(int2_status_result & BMI270_WRIST_GEST_STATUS_MASK){
                        bmi270_get_feature_data(&sens_data, 1, &bmi);
                        if (sens_data.sens_data.wrist_gesture_output == 3){
                            //shake -> enter interval setting mode
                            interval_setting_mode = 1;
                            buzz_x = 6;
                            buzz_en = 1;
                        }
                    }

                    int2_status_result = 0;
                }


                if(int1_status == 1){
                    int1_status = 0;
                    bmi2_get_int_status(&int1_status_result, &bmi);
                    /* To check the interrupt status of no-motion. */
                    if (int1_status_result & BMI270_NO_MOT_STATUS_MASK){
                        //printf("no-motion interrupt is generated\n");
                        int indx = 0;
                        Frame f[5];
                        while (indx < 5)
                        {
                            rslt = bmi2_get_sensor_data(&sensor_data_temp, &bmi);
                            if ((rslt == BMI2_OK) && (sensor_data_temp.status & BMI2_DRDY_ACC) &&
                                (sensor_data_temp.status & BMI2_DRDY_GYR))
                            {
                                f[indx] = (struct Frame){
                                    .acc = (struct Vec3d){
                                        .x = sensor_data_temp.acc.x,
                                        .y = sensor_data_temp.acc.y,
                                        .z = sensor_data_temp.acc.z
                                    },
                                    .gyr = (struct Vec3d){
                                        .x = sensor_data_temp.gyr.x,
                                        .y = sensor_data_temp.gyr.y,
                                        .z = sensor_data_temp.gyr.z
                                    }
                                };

                                indx++;
                            }
                        }

                        indx = 0;
                        switch (tree_classify(f)) {
                            case STAND:
                                alarm_en = 0;
                                break;
                            case SIT:
                                Timer_B_startCounter(TIMER_B0_BASE, TIMER_B_UP_MODE);
                                break;
                            case OTHER:
                                break;
                        }
                    }

                    int1_status_result = 0;
                }
            }

        } while(1);
    }
}

void init_spi() {
    WDTCTL = WDTPW | WDTHOLD;                 // Stop watchdog timer
    //SYSCFG3|=USCIA1RMP;                       //Set the remapping source
    P2SEL0 |= BIT4 | BIT5 | BIT6;             // set 4-SPI pin as second function
    //P3SEL0 |= BIT7;
    P3DIR |= BIT7;

    P3OUT &= ~BIT7;
    __delay_cycles(100);
    P3OUT |= BIT7;



    UCA1CTLW0 |= UCSWRST;                     // **Put state machine in reset**
                                              // 4-pin, 8-bit SPI master
    UCA1CTLW0 |= UCMST|UCSYNC|UCCKPL|UCMSB|UCMODE_1|UCSTEM;
                                              // Clock polarity high, MSB
    UCA1CTLW0 |= UCSSEL__ACLK;                // Select ACLK
    UCA1BR0 = 0x02;                           // BRCLK = ACLK/2
    UCA1BR1 = 0;                              //
    UCA1MCTLW = 0;                            // No modulation
    UCA1CTLW0 &= ~UCSWRST;                    // **Initialize USCI state machine**
    UCA1IE |= UCRXIE;
    //TXData = 0x80;                            // Holds TX data

    PM5CTL0 &= ~LOCKLPM5;                     // Disable the GPIO power-on default high-impedance mode
                                              // to activate previously configured port settings
}

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];
    #define ACCEL 0
    #define GYRO 1
    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_25HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_POWER_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_25HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_POWER_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
    }

    return rslt;
}


//P3.1
#pragma vector = PORT3_VECTOR
__interrupt void PORT3_ISR(void) {
    switch(__even_in_range(P3IV, P3IV_P3IFG7)){
        case P3IV_P3IFG1 :
            GPIO_disableInterrupt(GPIO_PORT_P3, GPIO_PIN1);
            int1_status = 1;
            __bic_SR_register_on_exit(LPM3_bits); // leave low power mode
            break;
        default:
            break;
    }
}

//P4.0
#pragma vector = PORT4_VECTOR
__interrupt void PORT4_ISR(void) {
    switch(__even_in_range(P4IV, P4IV_P4IFG7)){
        case P4IV_P4IFG0 :
            GPIO_disableInterrupt(GPIO_PORT_P4, GPIO_PIN0);
            int2_status = 1;
            __bic_SR_register_on_exit(LPM3_bits); // leave low power mode
            break;
        default:
            break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A1_VECTOR)))
#endif
void TIMER1_A1_ISR (void)
{
    //Any access, read or write, of the TAIV register automatically resets the
    //highest "pending" interrupt flag
    switch ( __even_in_range(TA1IV,14) ){
        case  0: break;                          //No interrupt
        case  2: break;                          //CCR1 not used
        case  4: break;                          //CCR2 not used
        case  6: break;                          //CCR3 not used
        case  8: break;                          //CCR4 not used
        case 10: break;                          //CCR5 not used
        case 12: break;                          //CCR6 not used
        case 14:
            if(buzz_en == 1 && counter < buzz_x){
                counter++;
                GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
            } else {
                buzz_en = 0;
                counter = 0;
            }
            break;
        default: break;
    }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_B1_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER0_B1_VECTOR)))
#endif
void TIMER0_B1_ISR(void)
{
    /* Any access, read or write, of the TBxIV register automatically resets the
       highest "pending" interrupt flag. */
    switch( __even_in_range(TB0IV,14) )
    {
        case TBIV__NONE:    break;      // No interrupt
        case TBIV__TBCCR1:  break;      // CCR1 not used
        case TBIV__TBCCR2:  break;      // CCR2 not used
        case TBIV__TBCCR3:  break;      // CCR3 not used
        case TBIV__TBCCR4:  break;      // CCR4 not used
        case TBIV__TBCCR5:  break;      // CCR5 not used
        case TBIV__TBCCR6:  break;      // CCR6 not used
        case TBIV__TBIFG:               // Overflow
           //interrupt every second
           if(interval_setting_mode == 0){
                timer_in_seconds++;

                if(timer_in_seconds > alarm_thres){
                    alarm_en = 1;
                    __bic_SR_register_on_exit(LPM3_bits); // leave low power mode
                }
           }

           if(interval_setting_mode == 1){
                __bic_SR_register_on_exit(LPM3_bits); // leave low power mode
           }
           break;
        default: break;
    }
}




