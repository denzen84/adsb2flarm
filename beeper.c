#include <stdio.h>
#include <unistd.h>
#include <stdbool.h>
#include <pthread.h>

#include "beeper.h"

const int sound_pattern[5][2] = { { 0, 1000},      // No sound
                             {100, 1000}, // Alarm level 1
                             {100, 400},  // Alarm level 2
                             {100, 50},   // Alarm level 3
                             {300, 50}     // Single beep
                          };


int beeper_init(struct beeper_gpio *beeper) {
  beeper->fd = fopen("/sys/class/gpio/export", "w");
  if(!beeper->fd)
  {
    fprintf(stderr, "[ERR] Error initializing beeper GPIO (/sys/class/gpio/export failed)\n");
    beeper->valid = false;
    return -1;
  }
  beeper->valid = fprintf(beeper->fd, "%d", beeper->gpio_pin) > 0;
  fclose(beeper->fd);


  char path[128];
  snprintf(path, 128, "/sys/class/gpio/gpio%d/direction", beeper->gpio_pin);

  beeper->fd = fopen(path, "w");

  if(!beeper->fd)
  {
	fprintf(stderr, "[ERR] Error initializing beeper PIN\n");
	beeper->valid = false;
    return -1;
  }
  beeper->valid = fprintf(beeper->fd, "%s", "out") > 0;
  fclose(beeper->fd);

  snprintf(path, 128, "/sys/class/gpio/gpio%d/value", beeper->gpio_pin);
  beeper->fd = fopen(path, "w");

  if(!beeper->fd)
  {
	fprintf(stderr, "[ERR] Error handle beeper PIN\n");
	beeper->valid = false;
	return -1;
  }
  setbuf(beeper->fd, NULL);

  if (!beeper->valid) {
		fprintf(stderr, "[ERR] General beeper error\n");
		return -1;
  }

  beeper->flarm_level = 0;
  beeper_mute(beeper);

  pthread_mutex_init(&beeper->bm, NULL);
  pthread_cond_init(&beeper->bcond, NULL);
  pthread_create(&beeper->thrd, NULL, beeper_flarm_thread_func, (void*) beeper);

  pthread_mutex_lock(&beeper->bm);
  pthread_cond_signal(&beeper->bcond);
  pthread_mutex_unlock(&beeper->bm);

  return 0;
}

void beeper_close(struct beeper_gpio *beeper) {
	fclose(beeper->fd);
}

void beeper_beep(struct beeper_gpio *beeper, int duration_ms) {
	if (!beeper->valid) return;
	fprintf(beeper->fd, "%d", ENABLE_BEEPER);
	usleep(duration_ms * 1000);
	fprintf(beeper->fd, "%d", DISABLE_BEEPER);
}

void beeper_mute(struct beeper_gpio *beeper) {
	if (!beeper->valid) return;
	fprintf(beeper->fd, "%d", DISABLE_BEEPER);
	fflush(beeper->fd);
}

void beeper_play_pattern(struct beeper_gpio *beeper) {
	if (!beeper->valid) return;
	if (beeper->flarm_level > 0) {
	  fprintf(beeper->fd, "%d", ENABLE_BEEPER);
	  usleep(sound_pattern[beeper->flarm_level][0] * 1000); // beep
	  fprintf(beeper->fd, "%d", DISABLE_BEEPER);
	  usleep(sound_pattern[beeper->flarm_level][1] * 1000); // mute
	} else beeper_mute(beeper);
}

void *beeper_flarm_thread_func(void *ptr) {
	struct beeper_gpio *beeper = (struct beeper_gpio *) ptr;
	while(true && beeper->valid) {

		pthread_mutex_lock(&beeper->bm);
		while(beeper->flarm_level == 0) {
			beeper_mute(beeper);
			pthread_cond_wait(&beeper->bcond, &beeper->bm);
		}
		pthread_mutex_unlock(&beeper->bm);
		beeper_play_pattern(beeper);
	}
	return NULL;
}

void beeper_play_flarm(struct beeper_gpio *beeper, int flarm_level) {

		if (flarm_level == beeper->flarm_level) return;

		pthread_mutex_lock(&beeper->bm);
		beeper->flarm_level = flarm_level;
		pthread_cond_signal(&beeper->bcond);
		pthread_mutex_unlock(&beeper->bm);
}

