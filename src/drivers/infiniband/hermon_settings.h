#ifndef HERMON_SETTINGS_H_
#define HERMON_SETTINGS_H_

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

FILE_LICENCE ( GPL2_OR_LATER );

char * get_device_name ( struct hermon *hermon );
extern void hermon_update_setting_ops ();
extern int hermon_init_port_settings ( struct hermon *hermon, struct hermon_port *port );
extern int hermon_init_settings ( struct hermon *hermon );
extern void hermon_restore_default ();

#endif /* HERMON_SETTINGS_H_*/
