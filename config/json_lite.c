#define _GNU_SOURCE
#include "json_lite.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct json_value {
    json_type_t type;
    union {
        bool   b;
        double n;
        char  *s;
        struct {
            json_value_t **items;
            size_t         count;
            size_t         cap;
        } arr;
        struct {
            char         **keys;
            json_value_t **vals;
            size_t         count;
            size_t         cap;
        } obj;
    };
};

typedef struct {
    const char *p;
    const char *end;
    int         depth;
} parser_t;

static void skip_ws(parser_t *ps)
{
    while (ps->p < ps->end &&
           (*ps->p == ' ' || *ps->p == '\t' || *ps->p == '\n' || *ps->p == '\r'))
        ps->p++;
}

static json_value_t *new_value(json_type_t t)
{
    json_value_t *v = calloc(1, sizeof(*v));
    if (v) v->type = t;
    return v;
}

static int parse_string_raw(parser_t *ps, char **out)
{
    if (ps->p >= ps->end || *ps->p != '"') return -1;
    ps->p++;

    size_t cap = 32, len = 0;
    char  *buf = malloc(cap);
    if (!buf) return -1;

    while (ps->p < ps->end) {
        char c = *ps->p;

        if (c == '"') {
            ps->p++;
            buf[len] = '\0';
            *out = buf;
            return 0;
        }
        if (c == '\\') {
            ps->p++;
            if (ps->p >= ps->end) { free(buf); return -1; }
            char e = *ps->p++;
            switch (e) {
            case '"':  c = '"';  break;
            case '\\': c = '\\'; break;
            case '/':  c = '/';  break;
            case 'b':  c = '\b'; break;
            case 'f':  c = '\f'; break;
            case 'n':  c = '\n'; break;
            case 'r':  c = '\r'; break;
            case 't':  c = '\t'; break;
            case 'u':
                /* 配置用不到 unicode，跳 4 位 hex，用 '?' 代替 */
                if (ps->end - ps->p < 4) { free(buf); return -1; }
                ps->p += 4;
                c = '?';
                break;
            default:
                free(buf);
                return -1;
            }
        } else {
            ps->p++;
        }

        if (len + 1 >= cap) {
            cap *= 2;
            char *nb = realloc(buf, cap);
            if (!nb) { free(buf); return -1; }
            buf = nb;
        }
        buf[len++] = c;
    }
    free(buf);
    return -1;
}

static json_value_t *parse_value(parser_t *ps);

static json_value_t *parse_object(parser_t *ps)
{
    json_value_t *v = new_value(JSON_OBJECT);
    if (!v) return NULL;
    ps->p++; /* { */
    skip_ws(ps);

    if (ps->p < ps->end && *ps->p == '}') { ps->p++; return v; }

    for (;;) {
        skip_ws(ps);
        char *key = NULL;
        if (parse_string_raw(ps, &key) < 0) { json_free(v); return NULL; }

        skip_ws(ps);
        if (ps->p >= ps->end || *ps->p != ':') {
            free(key); json_free(v); return NULL;
        }
        ps->p++;
        skip_ws(ps);

        json_value_t *val = parse_value(ps);
        if (!val) { free(key); json_free(v); return NULL; }

        if (v->obj.count == v->obj.cap) {
            size_t nc = v->obj.cap ? v->obj.cap * 2 : 8;
            char **nk = realloc(v->obj.keys, nc * sizeof(char *));
            json_value_t **nv = realloc(v->obj.vals, nc * sizeof(json_value_t *));
            if (!nk || !nv) { free(key); json_free(val); json_free(v); return NULL; }
            v->obj.keys = nk;
            v->obj.vals = nv;
            v->obj.cap  = nc;
        }
        v->obj.keys[v->obj.count] = key;
        v->obj.vals[v->obj.count] = val;
        v->obj.count++;

        skip_ws(ps);
        if (ps->p >= ps->end) { json_free(v); return NULL; }
        if (*ps->p == ',') { ps->p++; continue; }
        if (*ps->p == '}') { ps->p++; return v; }

        json_free(v);
        return NULL;
    }
}

static json_value_t *parse_array(parser_t *ps)
{
    json_value_t *v = new_value(JSON_ARRAY);
    if (!v) return NULL;
    ps->p++; /* [ */
    skip_ws(ps);

    if (ps->p < ps->end && *ps->p == ']') { ps->p++; return v; }

    for (;;) {
        skip_ws(ps);
        json_value_t *val = parse_value(ps);
        if (!val) { json_free(v); return NULL; }

        if (v->arr.count == v->arr.cap) {
            size_t nc = v->arr.cap ? v->arr.cap * 2 : 8;
            json_value_t **ni = realloc(v->arr.items, nc * sizeof(json_value_t *));
            if (!ni) { json_free(val); json_free(v); return NULL; }
            v->arr.items = ni;
            v->arr.cap   = nc;
        }
        v->arr.items[v->arr.count++] = val;

        skip_ws(ps);
        if (ps->p >= ps->end) { json_free(v); return NULL; }
        if (*ps->p == ',') { ps->p++; continue; }
        if (*ps->p == ']') { ps->p++; return v; }

        json_free(v);
        return NULL;
    }
}

static json_value_t *parse_value(parser_t *ps)
{
    if (++ps->depth > 32) { ps->depth--; return NULL; }
    skip_ws(ps);
    if (ps->p >= ps->end) { ps->depth--; return NULL; }

    char c = *ps->p;
    json_value_t *v = NULL;

    if (c == '{') {
        v = parse_object(ps);
    } else if (c == '[') {
        v = parse_array(ps);
    } else if (c == '"') {
        char *s = NULL;
        if (parse_string_raw(ps, &s) == 0) {
            v = new_value(JSON_STRING);
            if (v) v->s = s; else free(s);
        }
    } else if (c == 't') {
        if (ps->end - ps->p >= 4 && memcmp(ps->p, "true", 4) == 0) {
            ps->p += 4;
            v = new_value(JSON_BOOL);
            if (v) v->b = true;
        }
    } else if (c == 'f') {
        if (ps->end - ps->p >= 5 && memcmp(ps->p, "false", 5) == 0) {
            ps->p += 5;
            v = new_value(JSON_BOOL);
            if (v) v->b = false;
        }
    } else if (c == 'n') {
        if (ps->end - ps->p >= 4 && memcmp(ps->p, "null", 4) == 0) {
            ps->p += 4;
            v = new_value(JSON_NULL);
        }
    } else if (c == '-' || (c >= '0' && c <= '9')) {
        char *endp = NULL;
        double d = strtod(ps->p, &endp);
        if (endp == ps->p) { ps->depth--; return NULL; }
        ps->p = endp;
        v = new_value(JSON_NUMBER);
        if (v) v->n = d;
    }

    ps->depth--;
    return v;
}

json_value_t *json_parse(const char *text)
{
    if (!text) return NULL;
    parser_t ps = { text, text + strlen(text), 0 };
    return parse_value(&ps);
}

void json_free(json_value_t *v)
{
    if (!v) return;
    switch (v->type) {
    case JSON_STRING:
        free(v->s);
        break;
    case JSON_ARRAY:
        for (size_t i = 0; i < v->arr.count; i++) json_free(v->arr.items[i]);
        free(v->arr.items);
        break;
    case JSON_OBJECT:
        for (size_t i = 0; i < v->obj.count; i++) {
            free(v->obj.keys[i]);
            json_free(v->obj.vals[i]);
        }
        free(v->obj.keys);
        free(v->obj.vals);
        break;
    default:
        break;
    }
    free(v);
}

json_type_t json_type(const json_value_t *v) { return v ? v->type : JSON_NULL; }

const json_value_t *json_object_get(const json_value_t *obj, const char *key)
{
    if (!obj || obj->type != JSON_OBJECT) return NULL;
    for (size_t i = 0; i < obj->obj.count; i++)
        if (strcmp(obj->obj.keys[i], key) == 0)
            return obj->obj.vals[i];
    return NULL;
}

size_t json_array_size(const json_value_t *arr)
{
    return (arr && arr->type == JSON_ARRAY) ? arr->arr.count : 0;
}

const json_value_t *json_array_get(const json_value_t *arr, size_t i)
{
    if (!arr || arr->type != JSON_ARRAY || i >= arr->arr.count) return NULL;
    return arr->arr.items[i];
}

const char *json_string(const json_value_t *v, const char *def)
{
    return (v && v->type == JSON_STRING) ? v->s : def;
}

double json_number(const json_value_t *v, double def)
{
    if (!v) return def;
    if (v->type == JSON_NUMBER) return v->n;
    if (v->type == JSON_STRING) {
        char *endp = NULL;
        double d = strtod(v->s, &endp);
        return endp != v->s ? d : def;
    }
    return def;
}

bool json_bool(const json_value_t *v, bool def)
{
    return (v && v->type == JSON_BOOL) ? v->b : def;
}

long json_long(const json_value_t *v, long def)
{
    if (!v) return def;
    if (v->type == JSON_NUMBER) return (long)v->n;
    if (v->type == JSON_STRING) {
        char *endp = NULL;
        long l = strtol(v->s, &endp, 0); /* base 0：支持 0x/0/十进制 */
        return endp != v->s ? l : def;
    }
    return def;
}