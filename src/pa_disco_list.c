/*
 * pamodbus-disco — slave list management
 * MIT License — see LICENSE file for details.
 */

#include "pamodbus-disco-internal.h"
#include <stdlib.h>  /* malloc, free, realloc */
#include <string.h>  /* memset, memmove */

/* ---------------------------------------------------------------------------
 * Slave record
 * ------------------------------------------------------------------------- */

pa_disco_slave_t *pa_disco_slave_new(uint8_t slave_id,
                                     const uint16_t *serialno,
                                     void *arg)
{
    pa_disco_slave_t *slave = (pa_disco_slave_t *)malloc(sizeof(pa_disco_slave_t));
    if (slave) {
        slave->slave_id = slave_id;
        slave->serialno.hword[0] = serialno[0];
        slave->serialno.hword[1] = serialno[1];
        slave->serialno.hword[2] = serialno[2];
        slave->arg = arg;
    }
    return slave;
}

void pa_disco_slave_delete(pa_disco_slave_t *slave)
{
    free(slave);
}

uint8_t pa_disco_slave_get_id(const pa_disco_slave_t *slave)
{
    return slave->slave_id;
}

const uint16_t *pa_disco_slave_get_serialno(const pa_disco_slave_t *slave)
{
    return slave->serialno.hword;
}

void *pa_disco_slave_get_arg(const pa_disco_slave_t *slave)
{
    return slave->arg;
}

/* ---------------------------------------------------------------------------
 * Slave list
 * ------------------------------------------------------------------------- */

void pa_disco_list_init(pa_disco_list_t *list)
{
    memset(list, 0, sizeof(*list));
}

void pa_disco_list_clear(pa_disco_list_t *list)
{
    if (list && list->slave) {
        for (int i = 0; i < list->count; i++) {
            pa_disco_slave_delete(list->slave[i]);
        }
        free(list->slave);
        memset(list, 0, sizeof(*list));
    }
}

int pa_disco_list_count(const pa_disco_list_t *list)
{
    return list->count;
}

bool pa_disco_list_insert(pa_disco_list_t *list, pa_disco_slave_t *slave,
                          int index)
{
    if (index < 0 || index > list->count)
        return false;

    pa_disco_slave_t **new_slave = (pa_disco_slave_t **)realloc(
        list->slave, sizeof(pa_disco_slave_t *) * (size_t)(list->count + 1));
    if (!new_slave)
        return false;

    list->slave = new_slave;
    list->count++;
    if (index < list->count - 1) {
        memmove(&list->slave[index + 1], &list->slave[index],
                sizeof(pa_disco_slave_t *) * (size_t)(list->count - 1 - index));
    }
    list->slave[index] = slave;
    return true;
}

bool pa_disco_list_append(pa_disco_list_t *list, pa_disco_slave_t *slave)
{
    return pa_disco_list_insert(list, slave, list->count);
}

pa_disco_slave_t *pa_disco_list_at(const pa_disco_list_t *list, int index)
{
    if (index < 0 || index >= list->count)
        return NULL;
    return list->slave[index];
}

int pa_disco_list_find_serialno(const pa_disco_list_t *list,
                                const uint16_t *serialno)
{
    for (int i = 0; i < list->count; i++) {
        const pa_disco_slave_t *s = list->slave[i];
        if (s->serialno.hword[0] == serialno[0] &&
            s->serialno.hword[1] == serialno[1] &&
            s->serialno.hword[2] == serialno[2]) {
            return i;
        }
    }
    return -1;
}

int pa_disco_list_set_slave_id(pa_disco_list_t *list, int index,
                               uint8_t slave_id)
{
    pa_disco_slave_t *slave = pa_disco_list_at(list, index);
    if (!slave)
        return -1;
    slave->slave_id = slave_id;
    return 0;
}