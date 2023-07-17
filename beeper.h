#define BEEPER_PIN 423
#define ENABLE_BEEPER (int) 1
#define DISABLE_BEEPER (int) 0

struct beeper_gpio {
	FILE *fd;
	int flarm_level;
	int gpio_pin;
	pthread_t thrd;
	pthread_mutex_t bm;
	pthread_cond_t bcond;
	bool valid;
};

void *beeper_flarm_thread_func(void *ptr);

int beeper_init(struct beeper_gpio *beeper);
void beeper_close(struct beeper_gpio *beeper);
void beeper_beep(struct beeper_gpio *beeper, int duration_ms);
void beeper_mute(struct beeper_gpio *beeper);
void beeper_play_pattern(struct beeper_gpio *beeper);
void beeper_play_flarm(struct beeper_gpio *beeper, int flarm_level);
