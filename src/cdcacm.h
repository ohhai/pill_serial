/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* This file implements a the USB Communications Device Class - Abstract
 * Control Model (CDC-ACM) as defined in CDC PSTN subclass 1.2.
 *
 * The device's unique id is used as the USB serial number string.
 */
#ifndef __CDCACM_H
#define __CDCACM_H

#include <libopencm3/usb/usbd.h>

/*
 * Reducing the packet size to 32 is not enogh to have it working correctly. It will work for one bridge but there will still be timing issues when using more bridges concurrently. There is no simple fix and I also wonted to have forth on one USB serial port optionally so I have created a new project some time ago to achieve these goals if anybody is interrested: mecrisp_pillserial.
 *
 * https://github.com/ivpri/mecrisp_pillserial
 *
 * https://github.com/satoshinm/pill_serial/issues/1
 * https://github.com/satoshinm/pill_serial/pull/11#issuecomment-582306197
 * */

#define CDCACM_PACKET_SIZE 	32

extern usbd_device *usbdev;

void cdcacm_init(void);
/* Returns current usb configuration, or 0 if not configured. */
int cdcacm_get_config(void);
int cdcacm_get_dtr(void);

#endif
