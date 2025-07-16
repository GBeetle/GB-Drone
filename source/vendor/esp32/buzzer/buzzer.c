/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "buzzer.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "sdkconfig.h"

#include "io_define.h"
#include "gb_timer.h"

#define c 261
#define d 294
#define e 329
#define f 349
#define g 391
#define gS 415
#define a 440
#define aS 455
#define b 466
#define cH 523
#define cSH 554
#define dH 587
#define dSH 622
#define eH 659
#define fH 698
#define fSH 740
#define gH 784
#define gSH 830
#define aH 880

#define GPIO_OUTPUT_SPEED LEDC_LOW_SPEED_MODE // back too old git commit :-(
//#define GPIO_OUTPUT_SPEED LEDC_HIGH_SPEED_MODE

void init_buzzer(void)
{
    //
}

void sound(int gpio_num,uint32_t freq,uint32_t duration) {

	ledc_timer_config_t timer_conf;
	timer_conf.speed_mode = GPIO_OUTPUT_SPEED;
	//timer_conf.bit_num    = LEDC_TIMER_10_BIT;
	timer_conf.timer_num  = LEDC_TIMER_0;
	timer_conf.freq_hz    = freq;
	ledc_timer_config(&timer_conf);

	ledc_channel_config_t ledc_conf;
	ledc_conf.gpio_num   = gpio_num;
	ledc_conf.speed_mode = GPIO_OUTPUT_SPEED;
	ledc_conf.channel    = LEDC_CHANNEL_0;
	ledc_conf.intr_type  = LEDC_INTR_DISABLE;
	ledc_conf.timer_sel  = LEDC_TIMER_0;
	ledc_conf.duty       = 0x0; // 50%=0x3FFF, 100%=0x7FFF for 15 Bit
	                            // 50%=0x01FF, 100%=0x03FF for 10 Bit
	ledc_channel_config(&ledc_conf);

	// start
    ledc_set_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0, 0x7F); // 12% duty - play here for your speaker or buzzer
    ledc_update_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0);
	GB_SleepMs(duration);
	// stop
    ledc_set_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0, 0);
    ledc_update_duty(GPIO_OUTPUT_SPEED, LEDC_CHANNEL_0);

}

// based on https://wiki.mikrotik.com/wiki/Super_Mario_Theme
void play_theme() {
	sound(BUZZER_IO,660,100);
	GB_SleepMs(150);
	sound(BUZZER_IO,660,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,660,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,510,100);
	GB_SleepMs(100);
	sound(BUZZER_IO,660,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,770,100);
	GB_SleepMs(550);
	sound(BUZZER_IO,380,100);
	GB_SleepMs(575);

	sound(BUZZER_IO,510,100);
	GB_SleepMs(450);
	sound(BUZZER_IO,380,100);
	GB_SleepMs(400);
	sound(BUZZER_IO,320,100);
	GB_SleepMs(500);
	sound(BUZZER_IO,440,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,480,80);
	GB_SleepMs(330);
	sound(BUZZER_IO,450,100);
	GB_SleepMs(150);
	sound(BUZZER_IO,430,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,380,100);
	GB_SleepMs(200);
	sound(BUZZER_IO,660,80);
	GB_SleepMs(200);
	sound(BUZZER_IO,760,50);
	GB_SleepMs(150);
	sound(BUZZER_IO,860,100);
	GB_SleepMs(300);
	sound(BUZZER_IO,700,80);
	GB_SleepMs(150);
	sound(BUZZER_IO,760,50);
	GB_SleepMs(350);
	sound(BUZZER_IO,660,80);
	GB_SleepMs(300);
	sound(BUZZER_IO,520,80);
	GB_SleepMs(150);
	sound(BUZZER_IO,580,80);
	GB_SleepMs(150);
	sound(BUZZER_IO,480,80);
	GB_SleepMs(500);
}
// based on http://processors.wiki.ti.com/index.php/Playing_The_Imperial_March#Code
// original composed by John Williams for the film Star Wars: The Empire Strikes Back
void play_march(uint8_t longplay) {

    sound(BUZZER_IO,a, 500);
    sound(BUZZER_IO,a, 500);
    sound(BUZZER_IO,a, 500);
    sound(BUZZER_IO,f, 350);
    sound(BUZZER_IO,cH, 150);
    sound(BUZZER_IO,a, 500);
    sound(BUZZER_IO,f, 350);
    sound(BUZZER_IO,cH, 150);
    sound(BUZZER_IO,a, 650);

    GB_SleepMs(150);
    //end of first bit

    sound(BUZZER_IO,eH, 500);
    sound(BUZZER_IO,eH, 500);
    sound(BUZZER_IO,eH, 500);
    sound(BUZZER_IO,fH, 350);
    sound(BUZZER_IO,cH, 150);
    sound(BUZZER_IO,gS, 500);
    sound(BUZZER_IO,f, 350);
    sound(BUZZER_IO,cH, 150);
    sound(BUZZER_IO,a, 650);

    GB_SleepMs(150);
    //end of second bit...

    sound(BUZZER_IO,aH, 500);
    sound(BUZZER_IO,a, 300);
    sound(BUZZER_IO,a, 150);
    sound(BUZZER_IO,aH, 400);
    sound(BUZZER_IO,gSH, 200);
    sound(BUZZER_IO,gH, 200);
    sound(BUZZER_IO,fSH, 125);
    sound(BUZZER_IO,fH, 125);
    sound(BUZZER_IO,fSH, 250);

    GB_SleepMs(250);

    sound(BUZZER_IO,aS, 250);
    sound(BUZZER_IO,dSH, 400);
    sound(BUZZER_IO,dH, 200);
    sound(BUZZER_IO,cSH, 200);
    sound(BUZZER_IO,cH, 125);
    sound(BUZZER_IO,b, 125);
    sound(BUZZER_IO,cH, 250);

    GB_SleepMs(250);

    sound(BUZZER_IO,f, 125);
    sound(BUZZER_IO,gS, 500);
    sound(BUZZER_IO,f, 375);
    sound(BUZZER_IO,a, 125);
    sound(BUZZER_IO,cH, 500);
    sound(BUZZER_IO,a, 375);
    sound(BUZZER_IO,cH, 125);
    sound(BUZZER_IO,eH, 650);

    //end of third bit... (Though it doesn't play well)
    //let's repeat it
    if (longplay>=1) {
		sound(BUZZER_IO,aH, 500);
		sound(BUZZER_IO,a, 300);
		sound(BUZZER_IO,a, 150);
		sound(BUZZER_IO,aH, 400);
		sound(BUZZER_IO,gSH, 200);
		sound(BUZZER_IO,gH, 200);
		sound(BUZZER_IO,fSH, 125);
		sound(BUZZER_IO,fH, 125);
		sound(BUZZER_IO,fSH, 250);

		GB_SleepMs(250);

		sound(BUZZER_IO,aS, 250);
		sound(BUZZER_IO,dSH, 400);
		sound(BUZZER_IO,dH, 200);
		sound(BUZZER_IO,cSH, 200);
		sound(BUZZER_IO,cH, 125);
		sound(BUZZER_IO,b, 125);
		sound(BUZZER_IO,cH, 250);

		GB_SleepMs(250);

		sound(BUZZER_IO,f, 250);
		sound(BUZZER_IO,gS, 500);
		sound(BUZZER_IO,f, 375);
		sound(BUZZER_IO,cH, 125);
		sound(BUZZER_IO,a, 500);
		sound(BUZZER_IO,f, 375);
		sound(BUZZER_IO,cH, 125);
		sound(BUZZER_IO,a, 650);
		//end of the song
    }
}
