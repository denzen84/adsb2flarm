#include "dump1090.h"
#include "net_io.h"
#include "net_io.c"
#include "uart_io.h"

#include <termios.h>

struct _flarm_io flarm_io;

int set_interface_attribs(int fd, int speed);
void set_blocking (int fd, int should_block);


static void send_flarm_heartbeat(struct net_service *service)
{
    static char *heartbeat_message = "$GPTXT,HEARTBEAT MESSAGE*50\r\n";
    char *data;
    int len = strlen(heartbeat_message);

    if (!service->writer)
        return;

    data = prepareWrite(service->writer, len);
    if (!data)
        return;

    memcpy(data, heartbeat_message, len);
    completeWrite(service->writer, data + len);
}

void makeFlarmNetOutputService(void) {
	flarm_io.flarm_net_service = serviceInit("FLARM TCP output", &flarm_io.flarm_net_out, send_flarm_heartbeat, READ_MODE_IGNORE, NULL, NULL);
	serviceListen(flarm_io.flarm_net_service, Modes.net_bind_address, flarm_io.net_output_flarm_ports);
}

struct net_service *makeNmeaSerialPriInputOutputService(read_fn read_handler)
{
    return serviceInit("ASCII NMEA serial IO primary", &flarm_io.flarm_serial_pri_out, send_flarm_heartbeat, READ_MODE_ASCII, "\n", read_handler);
}

struct net_service *makeNmeaSerialSecInputOutputService(read_fn read_handler)
{
    return serviceInit("ASCII NMEA serial IO secondary", &flarm_io.flarm_serial_sec_out, send_flarm_heartbeat, READ_MODE_ASCII, "\n", read_handler);
}


struct net_service *makeNmeaSerialInputService(read_fn read_handler)
{
    return serviceInit("ASCII NMEA input", NULL, NULL, READ_MODE_ASCII, "\n", read_handler);
}

void writeFlarmOutput(struct net_service *service, const char *nmea_message) {
    char *data;
    int len = strlen(nmea_message);
    if (!service) return;
    if (!service->writer)
        return;
    data = prepareWrite(service->writer, len);
    if (!data)
        return;
    memcpy(data, nmea_message, len);
    completeWrite(service->writer, data + len);
}

int set_interface_attribs(int fd, int speed) {
	struct termios tty;

	if (tcgetattr(fd, &tty) < 0) {
		fprintf(stderr, "[ERR] Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}

	cfsetospeed(&tty, (speed_t) speed);
	cfsetispeed(&tty, (speed_t) speed);

	tty.c_cflag |= CLOCAL | CREAD;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8; /* 8-bit characters */
	tty.c_cflag &= ~PARENB; /* no parity bit */
	tty.c_cflag &= ~CSTOPB; /* only need 1 stop bit */
	tty.c_cflag &= ~CRTSCTS; /* no hardware flowcontrol */

	tty.c_lflag |= ICANON | ISIG; /* canonical input */
	tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | IEXTEN);

	tty.c_iflag &= ~IGNCR; /* preserve carriage return */
	tty.c_iflag &= ~INPCK;
	tty.c_iflag &= ~(INLCR | ICRNL | IUCLC | IMAXBEL);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY); /* no SW flowcontrol */

	tty.c_oflag &= ~OPOST;

	tty.c_cc[VEOL] = 0;
	tty.c_cc[VEOL2] = 0;
	tty.c_cc[VEOF] = 0x04;

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		fprintf(stderr, "[ERR] Error from tcgetattr: %s\n", strerror(errno));
		return -1;
	}
	return 0;
}

void set_blocking (int fd, int should_block)
{
        struct termios tty;
        memset (&tty, 0, sizeof tty);
        if (tcgetattr (fd, &tty) != 0)
        {
        	fprintf(stderr, "[WARN] Error %d from tggetattr", errno);
                return;
        }

        tty.c_cc[VMIN]  = should_block ? 1 : 0;
        tty.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

        if (tcsetattr (fd, TCSANOW, &tty) != 0)
        	fprintf(stderr, "[WARN] Error %d setting term attributes", errno);
}

struct client *createSerialClient(struct net_service *service, char *port) {
    struct client *c;

    if (!(c = (struct client *) malloc(sizeof(*c)))) {
        fprintf(stderr, "[ERR] Out of memory allocating a new %s serial client %s\n", service->descr, port);
        return NULL;
    }
    c->fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
    if (c->fd < 0) {
        fprintf(stderr, "[ERR] Error while initializing serial port %s\n", port);
        return NULL;
    }

    /*baudrate 115200, 8 bits, no parity, 1 stop bit */
    if (set_interface_attribs(c->fd, B115200) != 0) {
        fprintf(stderr, "[WARN] set_interface_attribs return unexpected code\n");
        return NULL;
    };
    set_blocking(c->fd, 0);

    c->service    = NULL;
    c->next       = Modes.clients;
    c->buflen     = 0;
    c->modeac_requested = 0;
    Modes.clients = c;

    moveNetClient(c, service);

    return c;
}
