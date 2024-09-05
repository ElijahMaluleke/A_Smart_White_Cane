/********************************************************************************
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 ********************************************************************************/

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_timer.h>
#include <nrfx_systick.h>
#include <zephyr/settings/settings.h>
#include <zephyr/logging/log.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <nrfx_saadc.h>

/********************************************************************************
 * The synchronization demo has two threads that utilize semaphores and sleeping
 * to take turns printing a greeting message at a controlled rate. The demo
 * shows both the static and dynamic approaches for spawning a thread; a real
 * world application would likely use the static approach for both threads.
 ********************************************************************************/

#define PIN_THREADS (IS_ENABLED(CONFIG_SMP) && IS_ENABLED(CONFIG_SCHED_CPU_MASK))

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 500

#define CONFIG_APP_VERSION	"1.0.0"

#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

#define BUTTON0_GPIO_PIN 		11
#define BUTTON1_GPIO_PIN		12
#define BUTTON2_GPIO_PIN		24
#define BUTTON3_GPIO_PIN		25

#define USER_BUTTON		 		BUTTON0_GPIO_PIN

//
#define LED0_GPIO_PIN 			13
#define LED1_GPIO_PIN			14
#define LED2_GPIO_PIN			15
#define LED3_GPIO_PIN			16
	
#define LED_ONE	 				LED0_GPIO_PIN
#define LED_TWO 				LED1_GPIO_PIN
#define LED_THREE 				LED2_GPIO_PIN
#define LED_FOUR 				LED3_GPIO_PIN

//
#define	AI_CAMERA_MODULE_PIN_1	3
#define	AI_CAMERA_MODULE_PIN_2	4

// TopSensor 
#define ECHO_TOP_SENSOR 		8 // Output pin
#define TRIG_TOP_SENSOR 		7 // Input pin
// Left Sensor
#define TRIG_LEFT_SENSOR 		6 // Output pin
#define ECHO_LEFT_SENSOR 		5 // Input pin
// Front Sensor
#define ECHO_FRONT_SENSOR 		4 // Output pin
#define TRIG_FRONT_SENSOR 		3 // Input pin
// Right Sensor
#define ECHO_RIGHT_SENSOR 		2 // Output pin
#define TRIG_RIGHT_SENSOR 		1 // Input pin

#define SENSORS_STATUS_LED	    LED_ONE
#define BAT_STATUS_LED          LED_TWO
#define AI_CAMERA_MODULE_LED	LED_THREE
#define RUN_LED_BLINK_INTERVAL  1000

#define BUZZER					28
#define VIBRATION_MOTOR			29
#define USER_LED                LED_THREE

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 			1000
#define SLEEP_TIME_S			1000
#define SLEEP_TIME_HALF_S		500
#define SLEEP_TIME_QUOTA_S		250

#define VIBRATION_MOTOR_ON_SLEEP_TIME	100
#define VIBRATION_MOTOR_OFF_SLEEP_TIME	50

#define  TOP_SENSOR			0x80
#define  LEFT_SENSOR 		0x81
#define  FRONT_SENSOR		0x82
#define  RIGHT_SENSOR		0x83

/* Option 1: by node label */
#define DEVICE_GPIO0 DT_NODELABEL(gpio0)
#define DEVICE_GPIO1 DT_NODELABEL(gpio1)

#define CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL	4

LOG_MODULE_REGISTER(a_smart_white_cane, CONFIG_A_SMART_WHITE_CANE_LOG_LEVEL);

// const struct device *gpio_dev;
const struct device *gpio0_dev = DEVICE_DT_GET(DEVICE_GPIO0);
// const struct device *gpio_dev;
const struct device *gpio1_dev = DEVICE_DT_GET(DEVICE_GPIO1);

/* Declare the struct to hold the configuration for the SAADC channel used to sample the battery voltage */
static nrfx_saadc_channel_t channel = NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0, 0);

/* Declare the buffer to hold the SAAD sample value */
static int16_t sample;

/* Define the battery sample interval */
#define BATTERY_SAMPLE_INTERVAL_MS 30000

/* Add forward declaration of timer callback handler */
static void battery_sample_timer_handler(struct k_timer *timer);

/* Define the battery sample timer instance */
K_TIMER_DEFINE(battery_sample_timer, battery_sample_timer_handler, NULL);


// counter
static volatile uint32_t tCount = 0;
static volatile uint32_t tCountTemp = 0;

// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;
static float dist_top;
static float dist_1eft;
static float dist_front;
static float dist_right;
static uint8_t prescaler = 1;
static uint16_t comp1 = 500;

// 
static int bat_volt = 0;
static bool bat_low_status = false;
static bool sensors_status = true;
static bool ai_cam_status = false;
//static uint8_t sensors = 0b10000111;
uint32_t dist_top_sensor = 0;
uint32_t dist_1eft_sensor = 0;
uint32_t dist_front_sensor = 0;
uint32_t dist_right_sensor = 0;

// 
static struct k_timer timer_sensors;
static struct gpio_callback ai_cam_cb_data;

/********************************************************************************
 *
 ********************************************************************************/
static bool getDistanceTopSensor(float* dist);
static bool getDistanceLeftSensor(float* dist);
static bool getDistanceFrontSensor(float* dist);
static bool getDistanceRightSensor(float* dist);
static void timer1_init(void);
static void set_conversion_factor(void);
static void MotorVibrations(uint32_t VibrationCount, uint8_t SleepTime);
static void ProximityOfObstacles(uint32_t Distance, uint8_t Sensor);
static void configure_saadc(void);

/********************************************************************************
 * set conversion factor
 ********************************************************************************/ 
static void set_conversion_factor(void)
{
	countToUs = 0.0625*comp1*(1 << prescaler);
}

/********************************************************************************
 *
 ********************************************************************************/
ISR_DIRECT_DECLARE(timer1_handler)
{
	if ((NRF_TIMER1->EVENTS_COMPARE[1]) && ((NRF_TIMER1->INTENSET) & (TIMER_INTENSET_COMPARE1_Msk)))
	{
		NRF_TIMER1->TASKS_CLEAR = 1;
		NRF_TIMER1->EVENTS_COMPARE[1] = 0;
		tCount++;
		//printk("Timer count: >%d<\n", tCount);
	}
	ISR_DIRECT_PM();
	return 1;
}

/********************************************************************************
 * Set up and start Timer1
  // set prescalar n
  // f = 16 MHz / 2^(n)

  // 16 MHz clock generates timer tick every 1/(16000000) s = 62.5 nano s
  // With compare enabled, the interrupt is fired every: 62.5 * comp1 nano s
  // = 0.0625*comp1 micro seconds
  // multiply this by 2^(prescalar)
 ********************************************************************************/
static void timer1_init(void)
{
	IRQ_DIRECT_CONNECT(TIMER1_IRQn, IRQ_PRIO_LOWEST, timer1_handler, 0);
	irq_enable(TIMER1_IRQn);
	NRF_TIMER1->MODE = TIMER_MODE_MODE_Timer;
	NRF_TIMER1->TASKS_CLEAR = 1;
	NRF_TIMER1->PRESCALER = prescaler;
	NRF_TIMER1->BITMODE = TIMER_BITMODE_BITMODE_16Bit;
	NRF_TIMER1->CC[1] = comp1;
	NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE1_CLEAR_Enabled << TIMER_SHORTS_COMPARE1_CLEAR_Pos;
	NRF_TIMER1->INTENSET = TIMER_INTENSET_COMPARE1_Enabled << TIMER_INTENSET_COMPARE1_Pos;
	set_conversion_factor();
	printf("timer tick = %f us\n", (double)countToUs);
	NRF_TIMER1->TASKS_START = 1;
}

/********************************************************************************
 *
 ********************************************************************************/
static void stop_timer(void)
{
	NRF_TIMER1->TASKS_STOP = 1;
}

/********************************************************************************
 * Stop Timer1
 ********************************************************************************/
static void start_timer(void)
{
	NRF_TIMER1->TASKS_START = 1;
}

/******************************************************************************** 
 * Implement timer callback handler function 
 ********************************************************************************/
void battery_sample_timer_handler(struct k_timer *timer)
{
    /* Trigger the sampling */
    nrfx_err_t err = nrfx_saadc_mode_trigger();
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

    /* Calculate and print voltage */
    int battery_voltage = ((600*6) * sample) / ((1<<12));
	bat_volt = battery_voltage;
	
    //printk("SAADC sample: %d\n", sample);
    printk("Battery Voltage: %d mV\n", bat_volt);
	bat_low_status = true;
}

/******************************************************************************** 
 * 
 ********************************************************************************/
static void configure_saadc(void)
{
    /* Connect ADC interrupt to nrfx interrupt handler */
    IRQ_CONNECT(DT_IRQN(DT_NODELABEL(adc)),
		    DT_IRQ(DT_NODELABEL(adc), priority),
		    nrfx_isr, nrfx_saadc_irq_handler, 0);
        
    /* Connect ADC interrupt to nrfx interrupt handler */
    nrfx_err_t err = nrfx_saadc_init(DT_IRQ(DT_NODELABEL(adc), priority));
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_mode_trigger error: %08x", err);
        return;
    }

    /* Configure the SAADC channel */
    channel.channel_config.gain = NRF_SAADC_GAIN1_6;
    err = nrfx_saadc_channels_config(&channel, 1);
    if (err != NRFX_SUCCESS) {
		printk("nrfx_saadc_channels_config error: %08x", err);
	    return;
	}

    /* Configure nrfx_SAADC driver in simple and blocking mode */
    err = nrfx_saadc_simple_mode_set(BIT(0),
                                         NRF_SAADC_RESOLUTION_12BIT,
                                         NRF_SAADC_OVERSAMPLE_DISABLED,
                                         NULL);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_simple_mode_set error: %08x", err);
            return;
    }
        
    /* Set buffer where sample will be stored */
    err = nrfx_saadc_buffer_set(&sample, 1);
    if (err != NRFX_SUCCESS) {
        printk("nrfx_saadc_buffer_set error: %08x", err);
        return;
    }

    /* Start periodic timer for battery sampling */
	k_timer_start(&battery_sample_timer, K_NO_WAIT, K_MSEC(BATTERY_SAMPLE_INTERVAL_MS));
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceTopSensor(float* dist)
{	
	sensors_status = false;
	gpio_pin_set(gpio1_dev, TRIG_TOP_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio1_dev, TRIG_TOP_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, ECHO_TOP_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, ECHO_TOP_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) {
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceLeftSensor(float* dist)
{
	sensors_status = false;
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, ECHO_LEFT_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, ECHO_LEFT_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) {
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceFrontSensor(float* dist)
{
	sensors_status = false;
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, ECHO_FRONT_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, ECHO_FRONT_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) {
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static bool getDistanceRightSensor(float* dist)
{
	sensors_status = false;
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, true);
	nrfx_systick_delay_us(10);
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, false);
	
	while(!gpio_pin_get(gpio1_dev, ECHO_RIGHT_SENSOR));
	// reset counter
	tCount = 0;
	while(gpio_pin_get(gpio1_dev, ECHO_RIGHT_SENSOR));
	
	float duration = countToUs*tCount;
	float distance = (double)duration*0.017;
	*dist = distance;
	sensors_status = true;
	if((double)distance < 400.0) {
		return true;
	}
	else {
		return false;
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static void MotorVibrations(uint32_t VibrationCount, uint8_t SleepTime)
{
	uint32_t i;
	
	for(i = 0; i < VibrationCount; i++) {
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, true);
		k_msleep(VIBRATION_MOTOR_ON_SLEEP_TIME * SleepTime);
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, false);
		k_msleep(VIBRATION_MOTOR_OFF_SLEEP_TIME);
	}
}

/********************************************************************************
 * @} 
 ********************************************************************************/
static void ProximityOfObstacles(uint32_t Distance, uint8_t Sensor)
{
	switch(Sensor){
		
		case TOP_SENSOR:
		case LEFT_SENSOR: 
		case FRONT_SENSOR:
		case RIGHT_SENSOR:
			if((Distance >= 0)&&(Distance <= 10)){
				MotorVibrations(1 , 18);
			}
			else if((Distance >= 10)&&(Distance <= 20)){
				MotorVibrations(1 , 16);
			}
			else if((Distance >= 20)&&(Distance <= 30)){
				MotorVibrations(1 , 14);
			}
			else if((Distance >= 30)&&(Distance <= 40)){
				MotorVibrations(1 , 12);
			}
			/*else if((Distance >= 40)&&(Distance <= 50)){
				MotorVibrations(1 , 10);
			}
			else if((Distance >= 60)&&(Distance <= 60)){
				MotorVibrations(1 , 8);
			}
			else if((Distance >= 70)&&(Distance <= 70)){
				MotorVibrations(1 , 6);
			}
			else if((Distance >= 80)&&(Distance <= 90)){
			MotorVibrations(1 , 4);
			}
			if((Distance >= 90)&&(Distance <= 100)){
				MotorVibrations(1 , 18);
			}*/
			else{
			}
		break;
	
		default:
		break;
	}	
}

/********************************************************************************
 * timer_sensors_exp_fnct
 ********************************************************************************/
static void timer_sensors_exp_fnct(struct k_timer *timer_id)
{
	printk("timer_sensors_expiry_function\n");
	if(sensors_status) {
		k_timer_start(&timer_sensors, K_MSEC(1000), K_NO_WAIT);
	}
	else{
		k_timer_start(&timer_sensors, K_MSEC(200), K_NO_WAIT);
	}
	gpio_pin_toggle(gpio0_dev, SENSORS_STATUS_LED);	
}

/********************************************************************************
 * Define the callback function
 ********************************************************************************/
void ai_cam_interrupt_handler(const struct device *dev, 
							  struct gpio_callback *cb, uint32_t pins){
	ai_cam_status = true;
	printk("\n\n\nAI_Cam Obects detected!\n");
}

/********************************************************************************
 * @param my_name      thread identification string
 * @param my_sem       thread's own semaphore
 * @param other_sem    other thread's semaphore
 ********************************************************************************/
void process_data_loop(const char *my_name,
		   struct k_sem *my_sem, struct k_sem *other_sem)
{
	const char *tname;
	uint8_t cpu;
	struct k_thread *current_thread;
	
	while (1) {
		/* take my semaphore */
		k_sem_take(my_sem, K_FOREVER);

		current_thread = k_current_get();
		tname = k_thread_name_get(current_thread);
		#if CONFIG_SMP
			cpu = arch_curr_cpu()->id;
		#else
			cpu = 0;
		#endif
		/* say "hello" */
		if (tname == NULL) {
			printk("%s: Hello World from cpu %d on %s!\n",
				my_name, cpu, CONFIG_BOARD);
		} else {
			printk("%s: Hello World from cpu %d on %s!\n",
				tname, cpu, CONFIG_BOARD);
		}

		/* wait a while, then let other thread have a turn */
		k_busy_wait(100000);
		k_msleep(SLEEPTIME);

		//
		if(getDistanceTopSensor(&dist_top)){
			dist_top_sensor = (uint32_t)dist_top;
			printf("Top Dist Sensor = %d cm\n", dist_top_sensor);
			ProximityOfObstacles(dist_top_sensor, TOP_SENSOR);
		}
		else {
			dist_top_sensor = (uint32_t)dist_top;
			printf("Top Distance < 400.0 %d cm\n", dist_top_sensor);
		}
		
		//
		if(getDistanceLeftSensor(&dist_1eft)){
			dist_1eft_sensor = (uint32_t)dist_1eft;
			printf("Left Dist Sensor = %d cm\n", dist_1eft_sensor);
			ProximityOfObstacles(dist_1eft_sensor, LEFT_SENSOR);
		}
		else {
			dist_1eft_sensor = (uint32_t)dist_1eft;
			printf("Left Distance < 400.0 %d cm\n", dist_1eft_sensor);
		}
		
		//
		if(getDistanceFrontSensor(&dist_front)){
			dist_front_sensor = (uint32_t)dist_front;
			printf("Front Dist Sensor = %d cm\n", dist_front_sensor);
			ProximityOfObstacles(dist_front_sensor, FRONT_SENSOR);
		}
		else {
			dist_front_sensor = (uint32_t)dist_front;
			printf("Front Distance < 400.0 %d cm\n", dist_front_sensor);
		}
		
		//
		if(getDistanceRightSensor(&dist_right)){
			dist_right_sensor = (uint32_t)dist_right;
			printf("Right Dist Sensor = %d cm\n", dist_right_sensor);
			ProximityOfObstacles(dist_right_sensor, RIGHT_SENSOR);
		}
		else {
			dist_right_sensor = (uint32_t)dist_right;
			printf("Right Distance < 400.0 %d cm\n", dist_right_sensor);
		}
		
		//
		if(ai_cam_status){
			ai_cam_status = false;
			gpio_pin_set(gpio0_dev, AI_CAMERA_MODULE_LED, false);
			MotorVibrations(1 , 6);
			gpio_pin_set(gpio0_dev, AI_CAMERA_MODULE_LED, true);
		}
		
		//
		if((bat_volt <= 1250) && (bat_low_status == true)){
			printf("Battery Low = %d mV\n", bat_volt);
			MotorVibrations(2 , 6);
			gpio_pin_set(gpio0_dev, BAT_STATUS_LED, false);
			k_msleep(250);
			gpio_pin_set(gpio0_dev, BAT_STATUS_LED, true);
			k_msleep(250);
			bat_low_status = false;
		}
		
		k_sem_give(other_sem);
	}
}

/* define semaphores */
K_SEM_DEFINE(thread_a_sem, 1, 1);	/* starts off "available" */
K_SEM_DEFINE(thread_b_sem, 0, 1);	/* starts off "not available" */

/********************************************************************************
 * thread_a is a dynamic thread that is spawned in main
 ********************************************************************************/
void thread_a_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* invoke routine to process data with thread_b */
	process_data_loop(__func__, &thread_a_sem, &thread_b_sem);
}
K_THREAD_STACK_DEFINE(thread_a_stack_area, STACKSIZE);
static struct k_thread thread_a_data;

/********************************************************************************
 * thread_b is a static thread spawned immediately
 ********************************************************************************/
void thread_b_entry_point(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);

	/* invoke routine to process data with thread_a */
	process_data_loop(__func__, &thread_b_sem, &thread_a_sem);
}
K_THREAD_DEFINE(thread_b, STACKSIZE,
				thread_b_entry_point, NULL, NULL, NULL,
				PRIORITY, 0, 0);
extern const k_tid_t thread_b;

/********************************************************************************
 * main.c - A Smart white Cane Application
 ********************************************************************************/
int main(void)
{
	//int err = 0;
	uint64_t i = 0;
	uint64_t j = 0;
	int blink_status = 0;
	
	tCount = 0;
	nrfx_systick_init();
	printk("\n\n\nA Smart white Cane Application started, version: %s\n", CONFIG_APP_VERSION);
	//
	printk("Timer1\n");
	timer1_init();
	
	// 
	if (!device_is_ready(gpio0_dev)) {
		return;
	}
	// 
	if (!device_is_ready(gpio1_dev)) {
		return;
	}
	
	/* set up TOP SENSOR TRIG ECHO pins */
	gpio_pin_configure(gpio1_dev, TRIG_TOP_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_TOP_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_set(gpio0_dev, TRIG_TOP_SENSOR, false);
	
	/* set up LEFT SENSOR TRIG ECHO pins */
	gpio_pin_configure(gpio1_dev, TRIG_LEFT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_LEFT_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_set(gpio1_dev, TRIG_LEFT_SENSOR, false);
	
	/* set up FRONT SENSOR TRIG ECHO pins */
	gpio_pin_configure(gpio1_dev, TRIG_FRONT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_FRONT_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_set(gpio1_dev, TRIG_FRONT_SENSOR, false);
	
	/* set up RIGHT SENSOR TRIG ECHO pins */
	gpio_pin_configure(gpio1_dev, TRIG_RIGHT_SENSOR, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio1_dev, ECHO_RIGHT_SENSOR, GPIO_INPUT | GPIO_ACTIVE_HIGH);	
	gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, false);
	
	// set up VIBRATION MOTOR pins
	gpio_pin_configure(gpio0_dev, VIBRATION_MOTOR, GPIO_OUTPUT_INACTIVE);	
	gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, false);
	
	// set up BUZZER pins
	gpio_pin_configure(gpio0_dev, BUZZER, GPIO_OUTPUT_INACTIVE);	
	gpio_pin_set(gpio0_dev, BUZZER, false);
	
	/* set up CAMERA MODULE pins */
	gpio_pin_configure(gpio0_dev, AI_CAMERA_MODULE_PIN_1, GPIO_OUTPUT_INACTIVE);
	gpio_pin_configure(gpio0_dev, AI_CAMERA_MODULE_PIN_2, GPIO_INPUT | GPIO_PULL_UP);	
	
	//
	gpio_pin_configure(gpio0_dev, LED_ONE, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_TWO, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_THREE, GPIO_OUTPUT_ACTIVE);
	gpio_pin_configure(gpio0_dev, LED_FOUR, GPIO_OUTPUT_ACTIVE);
	
	/* Configure the interrupt on the reed switch's pin */
	gpio_pin_interrupt_configure(gpio0_dev, AI_CAMERA_MODULE_PIN_2, GPIO_INT_LEVEL_INACTIVE);

	/* Initialize the static struct gpio_callback variable */
	gpio_init_callback(&ai_cam_cb_data, ai_cam_interrupt_handler, BIT(AI_CAMERA_MODULE_PIN_2));
	/* Add the callback function by calling gpio_add_callback() */
	gpio_add_callback(gpio0_dev, &ai_cam_cb_data);
	
	//
	k_msleep(SLEEP_TIME_MS * 2);

	//
	for(i = 0; i < 1; i++) {
		gpio_pin_set(gpio0_dev, LED_ONE, false);
		gpio_pin_set(gpio0_dev, LED_TWO, false);
		gpio_pin_set(gpio0_dev, LED_THREE, false);
		gpio_pin_set(gpio0_dev, LED_FOUR, false);
		gpio_pin_set(gpio0_dev, BUZZER, true);
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, true);
		gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, true);
		for(j = 0; j < 1000; j++) {
			nrfx_systick_delay_us(1000);
		}
		gpio_pin_set(gpio0_dev, LED_ONE, true);
		gpio_pin_set(gpio0_dev, LED_TWO, true);
		gpio_pin_set(gpio0_dev, LED_THREE, true);
		gpio_pin_set(gpio0_dev, LED_FOUR, true);
		gpio_pin_set(gpio0_dev, BUZZER, false);
		gpio_pin_set(gpio0_dev, VIBRATION_MOTOR, false);
		gpio_pin_set(gpio1_dev, TRIG_RIGHT_SENSOR, false);
		for(j = 0; j < 1000; j++) {
			nrfx_systick_delay_us(1000);
		}
	}
	
	//
	printk("Init timers\n");
	k_timer_init(&timer_sensors, timer_sensors_exp_fnct, NULL);
	k_timer_start(&timer_sensors, K_MSEC(1000), K_NO_WAIT);

	//
	k_thread_create(&thread_a_data, thread_a_stack_area,
			K_THREAD_STACK_SIZEOF(thread_a_stack_area),
			thread_a_entry_point, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&thread_a_data, "thread_a");

	#if PIN_THREADS
		if (arch_num_cpus() > 1) {
			k_thread_cpu_pin(&thread_a_data, 0);
			k_thread_cpu_pin(thread_b, 1);
		}
	#endif

	configure_saadc();
	k_thread_start(&thread_a_data);
	
	return 1;
}

/********************************************************************************
 * 
 ********************************************************************************/