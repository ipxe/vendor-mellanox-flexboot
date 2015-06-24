/*
 * Copyright (C) 2014 Mellanox Technologies Ltd.
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

FILE_LICENCE ( GPL2_OR_LATER );

#include <stdlib.h>
#include <errno.h>
#include <ipxe/status_updater.h>

/** @file
 *
 * Status updater
 *
 */

/** List of status updaters */
struct list_head status_updaters = LIST_HEAD_INIT ( status_updaters );

struct status_updater {
	struct list_head list;
	void *priv;
	status_update_fn update;
};

void status_update ( uint32_t status ) {
	struct status_updater *updater;
	list_for_each_entry ( updater, &status_updaters, list ) {
		if ( updater->update )
			updater->update ( updater->priv, status );
	}
}

int status_update_register_device ( void *priv, status_update_fn fn ) {
	struct status_updater *updater;

	updater = zalloc ( sizeof ( *updater ) );
	if ( ! updater )
		return -ENOMEM;

	updater->priv = priv;
	updater->update = fn;
	list_add ( & ( ( updater )->list ), &status_updaters );
	return 0;
}

void status_update_unregister_device ( void *priv ) {
	struct status_updater *updater, *tmp;
	list_for_each_entry_safe ( updater, tmp, &status_updaters, list ) {
		if ( updater->priv == priv ) {
			list_del ( & ( ( updater )->list ) );
			free ( updater );
			return;
		}
	}
}

