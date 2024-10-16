/*
 * This file is part of GB-Drone project (https://github.com/GBeetle/GB-Drone).
 * Copyright (c) 2022 GBeetle.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include "log_sys.h"

void app_main(void)
{
    int i = 0;
    printf("Hello world!\n");
    GB_DEBUGE(GB_INFO, "Hello world! [%d]", i++);
    GB_DEBUGW(GB_INFO, "Hello world! [%d]", i++);
    GB_DEBUGI(GB_INFO, "Hello world! [%d]", i++);
    GB_DEBUGD(GB_INFO, "Hello world! [%d]", i++);
    GB_DEBUGV(GB_INFO, "Hello world! [%d]", i++);

    return;
}
