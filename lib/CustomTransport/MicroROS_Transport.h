
#ifndef SRT_MICRO_ROS_TRANSPORT
#define SRT_MICRO_ROS_TRANSPORT

#include <Ethernet.h>
#include <uxr/client/transport.h>

// In GNU C < 6.0.0 __attribute__((deprecated(msg))) is not supported for enums, used in rmw/types.h
#if __GNUC__ < 6
#define aux__attribute__(x) __attribute__(x)
#define __attribute__(x)
#endif

#include <rmw_microros/rmw_microros.h>

#if __GNUC__ < 6
#undef __attribute__
#define __attribute__(x) aux__attribute__(x)
#endif

#ifdef __cplusplus
extern "C"
{
#endif

bool platformio_transport_open(struct uxrCustomTransport * transport);
bool platformio_transport_close(struct uxrCustomTransport * transport);
size_t platformio_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t platformio_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif


struct micro_ros_agent_locator {
	IPAddress address;
	int port;
};

static inline void set_microros_eth_transports(byte mac[], IPAddress client_ip, IPAddress agent_ip, uint16_t agent_port){

	static struct micro_ros_agent_locator locator;

   	Ethernet.begin(mac, client_ip);
	delay(1000);

	locator.address = agent_ip;
	locator.port = agent_port;

	rmw_uros_set_custom_transport(
		false,
		&locator,
		platformio_transport_open,
		platformio_transport_close,
		platformio_transport_write,
		platformio_transport_read
	);
}


#endif  // SRT_MICRO_ROS_TRANSPORT