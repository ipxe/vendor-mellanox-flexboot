/*
 * Copyright (C) 2014-2015 Mellanox Technologies Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#ifndef _IPXE_STATUS_UPDATER_H
#define _IPXE_STATUS_UPDATER_H

FILE_LICENCE ( GPL2_OR_LATER );

#include <ipxe/list.h>

typedef void ( * status_update_fn ) ( void *priv, uint8_t status );

/** Status stages */
#define STATUS_UPDATE_WAIT_ON_LINKUP	1
#define	STATUS_UPDATE_URI_BOOT			2
#define	STATUS_UPDATE_INT13_START		3
#define	STATUS_UPDATE_INT13_END			4

/** Send status update to who is registered in the list */
void status_update ( uint32_t status );
/** Register device in the list */
int status_update_register_device ( void *priv, status_update_fn fn );
/** Unregister device from the list */
void status_update_unregister_device ( void *priv );

#endif /* _IPXE_STATUS_UPDATER_H */
