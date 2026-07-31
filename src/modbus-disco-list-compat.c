/*
 * modbus-disco-list-compat.c — Compatibility implementation for modbus-disco list API
 *
 * This file provides the old modbus-disco list API functions for backward
 * compatibility with existing code that uses modbus_slave_t and modbus_disco_list_t.
 *
 * MIT License — see LICENSE file for details.
 */

#include <modbus-disco-list.h>
#include <stdlib.h>  /* malloc, free, realloc */
#include <string.h>  /* memset, memmove */

/* ---------------------------------------------------------------------------
 * Slave record management
 * ------------------------------------------------------------------------- */

modbus_slave_t* modbus_disco_slave_new(uint8_t slave_id,
                                       serialno_t* serialno,
                                       void* arg)
{
    modbus_slave_t* slave = (modbus_slave_t*)malloc(sizeof(modbus_slave_t));
    if (slave) {
        slave->slave_id = slave_id;
        if (serialno) {
            serialno_copy(&slave->serialno, serialno);
        } else {
            memset(&slave->serialno, 0, sizeof(serialno_t));
        }
        slave->arg = arg;
    }
    return slave;
}

void modbus_disco_slave_delete(modbus_slave_t* slave)
{
    free(slave);
}

/* ---------------------------------------------------------------------------
 * Slave list management
 * ------------------------------------------------------------------------- */

void modbus_disco_list_new(modbus_disco_list_t* list)
{
    memset(list, 0, sizeof(modbus_disco_list_t));
}

void modbus_disco_list_delete(modbus_disco_list_t* list)
{
    if (list && list->slave) {
        for (int n = 0; n < list->count; n++) {
            modbus_slave_t* slave = modbus_disco_list_at(list, n);
            if (slave) {
                modbus_disco_slave_delete(slave);
            }
        }
        free(list->slave);
        memset(list, 0, sizeof(modbus_disco_list_t));
    }
}

int modbus_disco_list_count(modbus_disco_list_t* list)
{
    return list->count;
}

bool modbus_disco_list_insert(modbus_disco_list_t* list,
                              modbus_slave_t* slave, int index)
{
    if (index < 0 || index > list->count)
        return false;

    modbus_slave_t** new_slave = (modbus_slave_t**)realloc(
        list->slave, sizeof(modbus_slave_t*) * (size_t)(list->count + 1));
    if (!new_slave)
        return false;

    list->slave = new_slave;
    list->count++;
    if (index < list->count - 1) {
        memmove(&list->slave[index + 1], &list->slave[index],
                sizeof(modbus_slave_t*) * (size_t)(list->count - 1 - index));
    }
    list->slave[index] = slave;
    return true;
}

bool modbus_disco_list_append(modbus_disco_list_t* list,
                              modbus_slave_t* slave)
{
    return modbus_disco_list_insert(list, slave, list->count);
}

modbus_slave_t* modbus_disco_list_at(modbus_disco_list_t* list, int index)
{
    if (index < 0 || index >= list->count)
        return NULL;
    return list->slave[index];
}

int modbus_disco_list_find_serialno(modbus_disco_list_t* list,
                                    serialno_t* serialno)
{
    for (int i = 0; i < list->count; i++) {
        modbus_slave_t* s = list->slave[i];
        if (serialno_equal(&s->serialno, serialno)) {
            return i;
        }
    }
    return -1;
}

int modbus_disco_list_set_slave_id(modbus_disco_list_t* list,
                                   int index, uint8_t slave_id)
{
    modbus_slave_t* slave = modbus_disco_list_at(list, index);
    if (!slave)
        return -1;
    slave->slave_id = slave_id;
    return 0;
}