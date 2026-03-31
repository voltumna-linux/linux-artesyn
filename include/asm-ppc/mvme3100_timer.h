#ifndef __MVME3100_TIMER_H
#define __MVME3100_TIMER_H


#define MVME3100_MIN_TIMER_NUMBER	0
#define MVME3100_MAX_TIMER_NUMBER	3

#define MIN_TICKS_PER_MICROSECOND	1
#define MAX_TICKS_PER_MICROSECOND	25

typedef void (*mvme3100_timer_f) (void *data);

/* 
 * Start a timer
 * Call func(data) after so many microsecoonds
 * The function will be called in interrupt context.
 */
int mvme3100_timer_start(int timer_number, unsigned long microseconds, 
			mvme3100_timer_f func, void *data, int periodic);

/* 
 * Stop a timer
 */
int mvme3100_timer_stop(int timer_number);

/* 
 * Get the timer resolution, in microseconds/tick
 */
unsigned long mvme3100_timer_get_resolution(void);

/* 
 * Set the timer resolution, in ticks/microseconds
 */
int mvme3100_timer_set_resolution(unsigned long ticks);

/* 
 * Get a timer counter value, in ticks
 */
unsigned long mvme3100_timer_get_ticks(int timer_number);

#endif /* __MVME3100_TIMER_H */
