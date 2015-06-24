#ifndef _USR_IFMGMT_H
#define _USR_IFMGMT_H

/** @file
 *
 * Network interface management
 *
 */

FILE_LICENCE ( GPL2_OR_LATER_OR_UBDL );

/** Default time to wait for link-up (in ticks per second) */
#define LINK_WAIT_TIMEOUT		( 20 * TICKS_PER_SEC )
/** Default time to wait for network to settle (in seconds) */
#define NETWORK_WAIT_TIMEOUT	35

struct net_device;
struct net_device_configurator;

extern int ifopen ( struct net_device *netdev );
extern int ifconf ( struct net_device *netdev,
		    struct net_device_configurator *configurator );
extern void ifclose ( struct net_device *netdev );
extern void ifstat ( struct net_device *netdev );
extern int iflinkwait ( struct net_device *netdev, unsigned long timeout );
extern int ifnetwork_wait ( struct net_device *netdev, unsigned long link_to,
					unsigned long network_to );
#endif /* _USR_IFMGMT_H */
