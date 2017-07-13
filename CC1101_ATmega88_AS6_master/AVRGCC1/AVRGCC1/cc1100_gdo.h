/*
 * Copyright  2008-2009 INRIA/SensTools
 * 
 * <dev-team@sentools.info>
 * 
 * This software is a set of libraries designed to develop applications
 * for the WSN430 embedded hardware platform.
 * 
 * This software is governed by the CeCILL license under French law and
 * abiding by the rules of distribution of free software.  You can  use, 
 * modify and/ or redistribute the software under the terms of the CeCILL
 * license as circulated by CEA, CNRS and INRIA at the following URL
 * "http://www.cecill.info". 
 * 
 * As a counterpart to the access to the source code and  rights to copy,
 * modify and redistribute granted by the license, users are provided only
 * with a limited warranty  and the software&apos;s author,  the holder of the
 * economic rights,  and the successive licensors  have only  limited
 * liability. 
 * 
 * In this respect, the user&apos;s attention is drawn to the risks associated
 * with loading,  using,  modifying and/or developing or reproducing the
 * software by the user in light of its specific status of free software,
 * that may mean  that it is complicated to manipulate,  and  that  also
 * therefore means  that it is reserved for developers  and  experienced
 * professionals having in-depth computer knowledge. Users are therefore
 * encouraged to load and test the software&apos;s suitability as regards their
 * requirements in conditions enabling the security of their systems and/or 
 * data to be ensured and,  more generally, to use and operate it in the 
 * same conditions as regards security. 
 * 
 * The fact that you are presently reading this means that you have had
 * knowledge of the CeCILL license and that you accept its terms.
 */



#ifndef _CC1100_GDO_H
#define _CC1100_GDO_H


#include "macro.h"


#define GDO0_PIN (1<<2)
#define GDO2_PIN (1<<3)

enum {
	LOW_EDGE=0,
	HIGH_EDGE
};



/*

#define GDO_INIT() do \
{ \
  PCSEL &= ~(GDO0_PIN | GDO2_PIN); \
  PCDDR &= ~(GDO0_PIN | GDO2_PIN); \
  P1IE  &= ~(GDO0_PIN | GDO2_PIN); \
} while (0)

*/


/*
#define GDO_INIT() do \
{ \
  PINC &= ~(GDO0_PIN | GDO2_PIN); \
  DDRC &= ~(GDO0_PIN | GDO2_PIN); \
  PCMSK1  |= (1<<PCINT10); \
  PCMSK1  |= (1<<PCINT11); \
} while (0)
*/



#define GDO_INIT() do \
{ \
  DDRC &= ~(GDO0_PIN | GDO2_PIN); \
  PCMSK1  |= (1<<PCINT10); \
  PCMSK1  |= (1<<PCINT11); \
} while (0)




//#define GDO0_INT_ENABLE() P1IE |= GDO0_PIN

#define GDO0_INT_ENABLE()   PCMSK1  |= (1<<PCINT10)

//#define GDO2_INT_ENABLE() P1IE |= GDO2_PIN

#define GDO2_INT_ENABLE()   PCMSK1  |= (1<<PCINT11)

//#define GDO0_INT_DISABLE() P1IE &= ~GDO0_PIN

#define GDO0_INT_DISABLE()  PCMSK1  &= ~(1<<PCINT10)

//#define GDO2_INT_DISABLE() P1IE &= ~GDO2_PIN

#define GDO2_INT_DISABLE()  PCMSK1  &= ~(1<<PCINT11)




//#define GDO0_INT_CLEAR() PCIFR &= ~(1<<PCIF1)
#define GDO0_INT_CLEAR() PCIFR |= (1<<PCIF1)

//#define GDO2_INT_CLEAR() PCIFR &= ~(1<<PCIF1)
#define GDO2_INT_CLEAR() PCIFR |= (1<<PCIF1)



#if 0

//#define GDO0_INT_SET_RISING()  P1IES &= ~GDO0_PIN
#define GDO0_INT_SET_RISING()  edge_val_gdo0 = HIGH_EDGE

//#define GDO0_INT_SET_FALLING() P1IES |=  GDO0_PIN
#define GDO0_INT_SET_FALLING() edge_val_gdo0 = LOW_EDGE

//#define GDO2_INT_SET_RISING()  P1IES &= ~GDO2_PIN
#define GDO2_INT_SET_RISING()  edge_val_gdo2 = HIGH_EDGE

//#define GDO2_INT_SET_FALLING() P1IES |=  GDO2_PIN
#define GDO2_INT_SET_FALLING() edge_val_gdo2 = LOW_EDGE

//#define GDO0_READ() (P1IN & GDO0_PIN)
#define GDO0_READ() (PINC & 0x04)

//#define GDO2_READ() (P1IN & GDO2_PIN)
#define GDO2_READ() (PINC & 0x08)

#endif












#define SENSOR_INT1_PIN (1<<3)


#define SENSOR_INIT() do \
{ \
  DDRD &= ~(SENSOR_INT1_PIN); \
  EIMSK  |= (1<<INT1); \
} while (0)


#define SENSOR_INT1_ENABLE()  EIMSK  |= (1<<INT1)
#define SENSOR_INT1_DISABLE()  EIMSK  &= ~(1<<INT1)




#define RTC_INT0_PIN (1<<2)

#define RTC_INIT() do \
{ \
  DDRD &= ~(RTC_INT0_PIN); \
  EIMSK  |= (1<<INT0); \
} while (0)


#define RTC_INT0_ENABLE()  EIMSK  |= (1<<INT0)
#define RTC_INT0_DISABLE() EIMSK  &= ~(1<<INT0)






#define RTC_INT0_CLEAR() EIFR &= ~(1<<INTF0)
#define SENSOR_INT1_CLEAR() EIFR &= ~(1<<INTF1)


#define rtc_int0_int_clear() \
	RTC_INT0_CLEAR()

#define sensor_int1_int_clear() \
	SENSOR_INT1_CLEAR()


#define RTC_INT0_SET_RISING() do \
{ \
	EICRA &= ~(1<<ISC00); \
	EICRA |= (1<<ISC01);  \
} while(0)


#define RTC_INT0_SET_FALLING() do \
{ \
	EICRA |= (1<<ISC00);  \
	EICRA |= (1<<ISC01);  \
} while(0)


#define SENSOR_INT1_SET_RISING() do \
{ \
	EICRA &= ~(1<<ISC10); \
	EICRA |= (1<<ISC11);  \
} while(0)


#define SENSOR_INT1_SET_FALLING() do \
{ \
	EICRA |= (1<<ISC10);  \
	EICRA |= (1<<ISC11);  \
} while(0)


#define rtc_int0_int_set_rising_edge() \
	RTC_INT0_SET_RISING()

#define rtc_int0_int_set_falling_edge() \
    RTC_INT0_SET_RISING()



#define sensor_int1_int_set_rising_edge() \
	SENSOR_INT1_SET_RISING()


#define sensor_int1_int_set_falling_edge() \
	SENSOR_INT1_SET_FALLING()






#endif




