/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 Dustin Fitzpatrick
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#include "config.h"
#include "lang.h"
#include "menu.h"
#include "settings.h"
#include "exported_menus.h"
#include "bluetooth/bt.h"

static int bluetooth_menu_callback(int action,
                                   const struct menu_item_ex *this_item,
                                   struct gui_synclist *this_list);
static int bluetooth_on_callback(int action,
                                 const struct menu_item_ex *this_item,
                                 struct gui_synclist *this_list);

MENUITEM_SETTING(bluetooth_enable, &global_settings.bluetooth, NULL);
MENUITEM_FUNCTION(bluetooth_paired_devices, 0,
                  ID2P(LANG_BLUETOOTH_PAIRED_DEVICES), bt_paired_devices,
                  bluetooth_on_callback, Icon_NOICON);
MENUITEM_FUNCTION(bluetooth_pair_new_device, 0,
                  ID2P(LANG_BLUETOOTH_PAIR_NEW_DEVICE), bt_pair_new_device,
                  bluetooth_on_callback, Icon_NOICON);
MAKE_MENU(bluetooth_settings_menu, ID2P(LANG_BLUETOOTH_SETTINGS),
          bluetooth_menu_callback, Icon_Config,
          &bluetooth_enable, &bluetooth_paired_devices,
          &bluetooth_pair_new_device);

/* Hide the whole menu on units without a Bluetooth controller */
static int bluetooth_menu_callback(int action,
                                   const struct menu_item_ex *this_item,
                                   struct gui_synclist *this_list)
{
    (void)this_list;
    if (action == ACTION_REQUEST_MENUITEM &&
        this_item == &bluetooth_settings_menu && !bt_is_available())
        return ACTION_EXIT_MENUITEM;
    return action;
}

/* The device screens are only shown while Bluetooth is on */
static int bluetooth_on_callback(int action,
                                 const struct menu_item_ex *this_item,
                                 struct gui_synclist *this_list)
{
    (void)this_item;
    (void)this_list;
    if (action == ACTION_REQUEST_MENUITEM && !global_settings.bluetooth)
        return ACTION_EXIT_MENUITEM;
    return action;
}
