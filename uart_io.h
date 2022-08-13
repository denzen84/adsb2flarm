//#include "net_io.h"

struct _flarm_io {
	struct net_service *flarm_net_service;
	struct net_writer flarm_net_out;
	char *net_output_flarm_ports;

	struct net_service *flarm_serial_service;
	struct net_writer flarm_serial_out;
};

extern struct _flarm_io flarm_io;

// FLARM via TCP
void makeFlarmNetOutputService(void);

// FLARM via serial port
struct client *createSerialClient(struct net_service *service, char *port); // Create IO serial client
struct net_service *makeNmeaSerialInputService(read_fn read_handler); // Serial input only (GNSS and other)
struct net_service *makeNmeaSerialInputOutputService(read_fn read_handler); // Serial IO service (FLARM and Tracker)

// Lagacy functions
void writeFlarmOutput(struct net_service *service, const char *nmea_message);



