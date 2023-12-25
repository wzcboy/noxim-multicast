#include "Routing_REC_BROADCAST.h"

RoutingAlgorithmsRegister Routing_REC_BROADCAST::routingAlgorithmsRegister("REC_BROADCAST", getInstance());

Routing_REC_BROADCAST * Routing_REC_BROADCAST::routing_rec_broadcast = 0;

Routing_REC_BROADCAST * Routing_REC_BROADCAST::getInstance() {
    if ( routing_rec_broadcast == 0 )
        routing_rec_broadcast = new Routing_REC_BROADCAST();

    return routing_rec_broadcast;
}

vector<int> Routing_REC_BROADCAST::XY_route(const RouteData & routeData) {
    Coord leftTop = routeData.p0;
    Coord rightDown = routeData.p1;
    Coord current = id2Coord(routeData.current_id);

    vector <int> directions;

    if (current.x < leftTop.x)
        directions.push_back(DIRECTION_EAST);
    else if (current.x > rightDown.x)
        directions.push_back(DIRECTION_WEST);
    else {
        if (current.y > rightDown.y)
            directions.push_back(DIRECTION_NORTH);
        else if (current.y < leftTop.y)
            directions.push_back(DIRECTION_SOUTH);
    }
    return directions;
}

bool isFirstInRec(Coord src, Coord current, Coord leftTop, Coord rightDown) {
    Coord nearest;
    if (src.x <= leftTop.x) {
        nearest.x = leftTop.x;
    } else if (src.x >= rightDown.x) {
        nearest.x = rightDown.x;
    } else {
        nearest.x = src.x;
    }

    if (src.y <= leftTop.y) {
        nearest.y = leftTop.y;
    } else if (src.y >= rightDown.y) {
        nearest.y = rightDown.y;
    } else {
        nearest.y = src.y;
    }
    return nearest == current;
}

vector<int> Routing_REC_BROADCAST::broadcast_route(const RouteData & routeData) {
    Coord leftTop = routeData.p0;
    Coord rightDown = routeData.p1;
    Coord current = id2Coord(routeData.current_id);
    Coord src = id2Coord(routeData.src_id);

    // 广播的每个数据包都发到本地PE
    vector <int> directions = {DIRECTION_LOCAL};

    int in_port = routeData.dir_in;
    // 第一次进入该矩形区域，则向所有输出端口发送
    if (isFirstInRec(src, current, leftTop, rightDown)) {
        if (current.y != leftTop.y) directions.push_back(DIRECTION_NORTH);
        if (current.y != rightDown.y) directions.push_back(DIRECTION_SOUTH);
        if (current.x != leftTop.x) directions.push_back(DIRECTION_WEST);
        if (current.x != rightDown.x) directions.push_back(DIRECTION_EAST);

        return directions;
    }
    if (in_port == DIRECTION_NORTH) {
        if (current.y != rightDown.y) directions.push_back(DIRECTION_SOUTH);
    } else if (in_port == DIRECTION_SOUTH) {
        if (current.y != leftTop.y) directions.push_back(DIRECTION_NORTH);
    } else if (in_port == DIRECTION_WEST) {
        if (current.y != leftTop.y) directions.push_back(DIRECTION_NORTH);
        if (current.y != rightDown.y) directions.push_back(DIRECTION_SOUTH);
        if (current.x != rightDown.x) directions.push_back(DIRECTION_EAST);
    } else if (in_port == DIRECTION_EAST) {
        if (current.y != leftTop.y) directions.push_back(DIRECTION_NORTH);
        if (current.y != rightDown.y) directions.push_back(DIRECTION_SOUTH);
        if (current.x != leftTop.x) directions.push_back(DIRECTION_WEST);
    } else if (in_port == DIRECTION_LOCAL) {
        directions.clear();
        if (current.y != leftTop.y) directions.push_back(DIRECTION_NORTH);
        if (current.y != rightDown.y) directions.push_back(DIRECTION_SOUTH);
        if (current.x != leftTop.x) directions.push_back(DIRECTION_WEST);
        if (current.x != rightDown.x) directions.push_back(DIRECTION_EAST);
    }

    return directions;
}

bool Routing_REC_BROADCAST::isCoordInRec(const Coord & p, const Coord & leftTop, const Coord & rightDown) {
    return (p.x <= rightDown.x && p.x >= leftTop.x && p.y >= leftTop.y && p.y <= rightDown.y);
}

vector<int> Routing_REC_BROADCAST::route(Router * router, const RouteData & routeData)
{
    vector <int> directions;

    if (routeData.packet_type == MYPACKET_TYPE_UNICAST) {
        directions = XY_route(routeData);
    }
    else if (routeData.packet_type == MYPACKET_TYPE_MULTICAST || routeData.packet_type == MYPACKET_TYPE_BROADCAST) {
        Coord current_coord = id2Coord(routeData.current_id);
        if (isCoordInRec(current_coord, routeData.p0, routeData.p1)) {
            directions = broadcast_route(routeData);
        } else {
            directions = XY_route(routeData);
        }
    }
    return directions;
}
