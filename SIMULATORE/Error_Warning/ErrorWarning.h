#ifndef ERRORWARNING_H
#define ERRORWARNING_H

#define WARNING(code, ...) Warning(code, __func__, ##__VA_ARGS__)
#define ERROR(code, ...) Error(code, __func__, ##__VA_ARGS__)

void Error(int code, const char *func_name, ...);
void Warning(int code, const char *func_name, ...);

void SetColor(int color);

#endif