#ifndef JSON_LITE_H
#define JSON_LITE_H

#include <stddef.h>
#include <stdbool.h>

typedef enum {
    JSON_NULL = 0, JSON_BOOL, JSON_NUMBER, JSON_STRING, JSON_ARRAY, JSON_OBJECT
} json_type_t;

typedef struct json_value json_value_t;

json_value_t *json_parse(const char *text);
void          json_free(json_value_t *v);

json_type_t   json_type(const json_value_t *v);
const json_value_t *json_object_get(const json_value_t *obj, const char *key);
size_t              json_array_size(const json_value_t *arr);
const json_value_t *json_array_get(const json_value_t *arr, size_t i);

const char *json_string(const json_value_t *v, const char *def);
double      json_number(const json_value_t *v, double def);
bool        json_bool(const json_value_t *v, bool def);
long        json_long(const json_value_t *v, long def);

#endif