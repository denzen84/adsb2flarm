
struct _flarm_io {
	struct net_service *flarm_net_service;
	struct net_writer flarm_net_out;
	char *net_output_flarm_ports;

	struct net_service *flarm_serial_pri_service;
	struct net_writer flarm_serial_pri_out;

	struct net_service *flarm_serial_sec_service;
	struct net_writer flarm_serial_sec_out;

};

extern struct _flarm_io flarm_io;

// FLARM via TCP
void makeFlarmNetOutputService(void);

// FLARM via serial port
struct client *createSerialClient(struct net_service *service, char *port); // Create IO serial client
struct net_service *makeNmeaSerialInputService(read_fn read_handler); // Serial input only (GNSS and other)
struct net_service *makeNmeaSerialPriInputOutputService(read_fn read_handler); // Serial IO service primary
struct net_service *makeNmeaSerialSecInputOutputService(read_fn read_handler); // Serial IO service secondary

// Legacy functions
void writeFlarmOutput(struct net_service *service, const char *nmea_message);



