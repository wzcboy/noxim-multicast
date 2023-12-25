//
// Created by wzc on 2023/4/13.
//

#ifndef NOXIM_ROUTING_REC_BROADCAST_H
#define NOXIM_ROUTING_REC_BROADCAST_H

#include "RoutingAlgorithm.h"
#include "RoutingAlgorithms.h"
#include "../Router.h"

using namespace std;

class Routing_REC_BROADCAST : RoutingAlgorithm {
public:
    vector<int> route(Router * router, const RouteData & routeData);

    static Routing_REC_BROADCAST * getInstance();

private:
    Routing_REC_BROADCAST(){};
    ~Routing_REC_BROADCAST(){};

    vector<int> XY_route(const RouteData & routeData);
    vector<int> broadcast_route(const RouteData & routeData);
    bool isCoordInRec(const Coord & p, const Coord & leftTop, const Coord & rightDown);

    static Routing_REC_BROADCAST * routing_rec_broadcast;
    static RoutingAlgorithmsRegister routingAlgorithmsRegister;
};

#endif //NOXIM_ROUTING_REC_BROADCAST_H
