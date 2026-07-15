#ifndef H_ROUTE_H
#define H_ROUTE_H

#include <stdint.h>


typedef enum {
    ROUTE_AB = 0,//路线1：A->B
    ROUTE_LOOP = 1,//路线2：完整一圈：A->B->C->D->A
    ROUTE_DIAGONAL = 2,//路线3：A->C->B->D->A
    ROUTE_DIAGONAL_4 = 3//路线3行驶4圈
} RouteProgram;

void Route_Select(RouteProgram program);
void Route_Start(float current_yaw_deg);
void Route_Update(float current_yaw_deg);
uint8_t Route_IsComplete(void);

#endif
