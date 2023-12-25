/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the processing element
 */

#include "ProcessingElement.h"

int ProcessingElement::randInt(int min, int max)
{
    return min +
	(int) ((double) (max - min + 1) * rand() / (RAND_MAX + 1.0));
}

void ProcessingElement::rxProcess()
{
    if (reset.read()) {
	ack_rx.write(0);
	current_level_rx = 0;
    } else {
	if (req_rx.read() == 1 - current_level_rx) {
	    MyPacket flit_tmp = flit_rx.read();
        // ...数据包暂时不作处理
	    current_level_rx = 1 - current_level_rx;	// Negate the old value for Alternating Bit Protocol (ABP)
	}
	ack_rx.write(current_level_rx);
    }
}

void ProcessingElement::txProcess()
{
    if (reset.read()) {
	req_tx.write(0);
	current_level_tx = 0;
	transmittedAtPreviousCycle = false;
    } else {
	MyPacket packet;

	if (canShot(packet)) {
	    packet_queue.push(packet);
	    transmittedAtPreviousCycle = true;
	} else
	    transmittedAtPreviousCycle = false;


	if (ack_tx.read() == current_level_tx) {
	    if (!packet_queue.empty()) {
		MyPacket pkt = packet_queue.front();	// Generate a new packet
        packet_queue.pop();
		flit_tx->write(pkt);	// Send the generated pkt
		current_level_tx = 1 - current_level_tx;	// Negate the old value for Alternating Bit Protocol (ABP)
		req_tx.write(current_level_tx);
	    }
	}
    }
}

//Flit ProcessingElement::nextFlit()
//{
//    Flit flit;
//    Packet packet = packet_queue.front();
//
//    flit.src_id = packet.src_id;
//    flit.dst_id = packet.dst_id;
//    flit.vc_id = packet.vc_id;
//    flit.timestamp = packet.timestamp;
//    flit.sequence_no = packet.size - packet.flit_left;
//    flit.sequence_length = packet.size;
//    flit.hop_no = 0;
//    //  flit.payload     = DEFAULT_PAYLOAD;
//
//    flit.hub_relay_node = NOT_VALID;
//
//    if (packet.size == packet.flit_left)
//	flit.flit_type = FLIT_TYPE_HEAD;
//    else if (packet.flit_left == 1)
//	flit.flit_type = FLIT_TYPE_TAIL;
//    else
//	flit.flit_type = FLIT_TYPE_BODY;
//
//    packet_queue.front().flit_left--;
//    if (packet_queue.front().flit_left == 0)
//	packet_queue.pop();
//
//    return flit;
//}

bool ProcessingElement::canShot(MyPacket & packet)
{
   // assert(false);
    if(never_transmit) return false;
   
    //if(local_id!=16) return false;
    /* DEADLOCK TEST 
	double current_time = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;

	if (current_time >= 4100) 
	{
	    //if (current_time==3500)
	         //cout << name() << " IN CODA " << packet_queue.size() << endl;
	    return false;
	}
	//*/

#ifdef DEADLOCK_AVOIDANCE
    if (local_id%2==0)
	return false;
#endif
    bool shot;
    double threshold;

    double now = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;

    if (GlobalParams::traffic_distribution != TRAFFIC_TABLE_BASED) {
	if (!transmittedAtPreviousCycle)
	    threshold = GlobalParams::packet_injection_rate;
	else
	    threshold = GlobalParams::probability_of_retransmission;

	shot = (((double) rand()) / RAND_MAX < threshold);
	if (shot) {
	    if (GlobalParams::traffic_distribution == TRAFFIC_RANDOM)
		    packet = trafficRandom();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_TRANSPOSE1)
//		    packet = trafficTranspose1();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_TRANSPOSE2)
//    		packet = trafficTranspose2();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_BIT_REVERSAL)
//		    packet = trafficBitReversal();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_SHUFFLE)
//		    packet = trafficShuffle();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_BUTTERFLY)
//		    packet = trafficButterfly();
//        else if (GlobalParams::traffic_distribution == TRAFFIC_LOCAL)
//		    packet = trafficLocal();
        else if (GlobalParams::traffic_distribution == TRAFFIC_ULOCAL)
		    packet = trafficULocal();
        else if (GlobalParams::traffic_distribution == TRAFFIC_MULTICAST)
            packet = trafficMulticast();
        else if (GlobalParams::traffic_distribution == TRAFFIC_TEST)
            packet = trafficTest();
        else {
            cout << "Invalid traffic distribution: " << GlobalParams::traffic_distribution << endl;
            exit(-1);
        }
	}
    } else {			// Table based communication traffic
//	if (never_transmit)
//	    return false;
//
//	bool use_pir = (transmittedAtPreviousCycle == false);
//	vector < pair < int, double > > dst_prob;
//	double threshold =
//	    traffic_table->getCumulativePirPor(local_id, (int) now, use_pir, dst_prob);
//
//	double prob = (double) rand() / RAND_MAX;
//	shot = (prob < threshold);
//	if (shot) {
//	    for (unsigned int i = 0; i < dst_prob.size(); i++) {
//		if (prob < dst_prob[i].second) {
//                    int vc = randInt(0,GlobalParams::n_virtual_channels-1);
//		    packet.make(local_id, dst_prob[i].first, vc, now, getRandomSize());
//		    break;
//		}
//	    }
//	}
    }
    // for debug
//    Coord tmp(0, 2);
//    if (packet.src_id == coord2Id(tmp))
//        return shot;
//    else return false;
    return shot;
}


Packet ProcessingElement::trafficLocal()
{
    Packet p;
    p.src_id = local_id;
    double rnd = rand() / (double) RAND_MAX;

    vector<int> dst_set;

    int max_id = (GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y);

    for (int i=0;i<max_id;i++)
    {
	if (rnd<=GlobalParams::locality)
	{
	    if (local_id!=i && sameRadioHub(local_id,i))
		dst_set.push_back(i);
	}
	else
	    if (!sameRadioHub(local_id,i))
		dst_set.push_back(i);
    }


    int i_rnd = rand()%dst_set.size();

    p.dst_id = dst_set[i_rnd];
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    
    return p;
}


int ProcessingElement::findRandomDestination(int id, int hops)
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);

    int inc_y = rand()%2?-1:1;
    int inc_x = rand()%2?-1:1;
    
    Coord current =  id2Coord(id);
    


    for (int h = 0; h<hops; h++)
    {

	if (current.x==0)
	    if (inc_x<0) inc_x=0;

	if (current.x== GlobalParams::mesh_dim_x-1)
	    if (inc_x>0) inc_x=0;

	if (current.y==0)
	    if (inc_y<0) inc_y=0;

	if (current.y==GlobalParams::mesh_dim_y-1)
	    if (inc_y>0) inc_y=0;

	if (rand()%2)
	    current.x +=inc_x;
	else
	    current.y +=inc_y;
    }
    return coord2Id(current);
}


int roulette()
{
    int slices = GlobalParams::mesh_dim_x + GlobalParams::mesh_dim_y -2;


    double r = rand()/(double)RAND_MAX;


    for (int i=1;i<=slices;i++)
    {
	if (r< (1-1/double(2<<i)))
	{
	    return i;
	}
    }
    assert(false);
    return 1;
}

// 找到比输入id大的节点
Coord findGreaterDestination(int id, int hops)
{
    int inc_y = 1;
    int inc_x = 1;
    Coord current = id2Coord(id);

    for (int i=0; i<hops; i++) {
        if (current.x== GlobalParams::mesh_dim_x-1)
            inc_x = 0;

        if (current.y==GlobalParams::mesh_dim_y-1)
            inc_y = 0;

        if (inc_x == 0) {
            current.y += inc_y;
        } else if (inc_y == 0) {
            current.x += inc_x;
        } else {
            if (rand()%2)
                current.x += inc_x;
            else
                current.y += inc_y;
        }

    }
    return current;
}

MyPacket ProcessingElement::trafficULocal()
{
    MyPacket p;

    p.packet_type = MYPACKET_TYPE_UNICAST;
    p.src_id = local_id;

    int target_hops = roulette();
    int dst_id = findRandomDestination(local_id,target_hops);
    p.p0 = p.p1 = id2Coord(dst_id);

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.hasRouted = false;

    return p;
}

MyPacket ProcessingElement::trafficMulticast()
{
    MyPacket p;

    p.packet_type = MYPACKET_TYPE_MULTICAST;
    p.src_id = local_id;

    int max_id = (GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y) - 1;
    int leftTop_id;
    do {
        leftTop_id = randInt(0, max_id);
    } while (leftTop_id == p.src_id);

    // 产生1~3的随机数
    int target_hops = (rand() % 3) + 1;
    Coord rightDown = findGreaterDestination(leftTop_id, target_hops);

    p.p0 = id2Coord(leftTop_id);
    p.p1 = rightDown;

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.hasRouted = false;
    for (int i=0; i<DIRECTIONS+2; i++) {
        p.out_ports[i] = 0;
    }
    return p;
}

MyPacket ProcessingElement::trafficRandom()
{
    MyPacket p;
    p.packet_type = MYPACKET_TYPE_UNICAST;
    p.src_id = local_id;
    double rnd = rand() / (double) RAND_MAX;
    double range_start = 0.0;
    int max_id;

    if (GlobalParams::topology == TOPOLOGY_MESH)
	max_id = (GlobalParams::mesh_dim_x * GlobalParams::mesh_dim_y) - 1; //Mesh 
    else    // other delta topologies
	max_id = GlobalParams::n_delta_tiles-1; 

    // Random destination distribution
    int dst_id;
    do {
	dst_id = randInt(0, max_id);

	// check for hotspot destination
	for (size_t i = 0; i < GlobalParams::hotspots.size(); i++) {

	    if (rnd >= range_start && rnd < range_start + GlobalParams::hotspots[i].second) {
		if (local_id != GlobalParams::hotspots[i].first ) {
		    dst_id = GlobalParams::hotspots[i].first;
		}
		break;
	    } else
		range_start += GlobalParams::hotspots[i].second;	// try next
	}
#ifdef DEADLOCK_AVOIDANCE
	assert((GlobalParams::topology == TOPOLOGY_MESH));
	if (p.dst_id%2!=0)
	{
	    p.dst_id = (p.dst_id+1)%256;
	}
#endif

    } while (dst_id == p.src_id);
    p.p0 = p.p1 = id2Coord(dst_id);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.hasRouted = false;
    for (int i=0; i<DIRECTIONS+2; i++) {
        p.out_ports[i] = 0;
    }
    return p;
}
// TODO: for testing only
MyPacket ProcessingElement::trafficTest()
{
    MyPacket p;

    p.packet_type = MYPACKET_TYPE_MULTICAST;
    p.src_id = local_id;
    p.p0 = Coord(3,0);
    p.p1 = Coord(3,1);

    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.hasRouted = false;
    for (int i=0; i<DIRECTIONS+2; i++) {
        p.out_ports[i] = 0;
    }
    return p;
}

Packet ProcessingElement::trafficTranspose1()
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);
    Packet p;
    p.src_id = local_id;
    Coord src, dst;

    // Transpose 1 destination distribution
    src.x = id2Coord(p.src_id).x;
    src.y = id2Coord(p.src_id).y;
    dst.x = GlobalParams::mesh_dim_x - 1 - src.y;
    dst.y = GlobalParams::mesh_dim_y - 1 - src.x;
    fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficTranspose2()
{
    assert(GlobalParams::topology == TOPOLOGY_MESH);
    Packet p;
    p.src_id = local_id;
    Coord src, dst;

    // Transpose 2 destination distribution
    src.x = id2Coord(p.src_id).x;
    src.y = id2Coord(p.src_id).y;
    dst.x = src.y;
    dst.y = src.x;
    fixRanges(src, dst);
    p.dst_id = coord2Id(dst);

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

void ProcessingElement::setBit(int &x, int w, int v)
{
    int mask = 1 << w;

    if (v == 1)
	x = x | mask;
    else if (v == 0)
	x = x & ~mask;
    else
	assert(false);
}

int ProcessingElement::getBit(int x, int w)
{
    return (x >> w) & 1;
}

inline double ProcessingElement::log2ceil(double x)
{
    return ceil(log(x) / log(2.0));
}

Packet ProcessingElement::trafficBitReversal()
{

    int nbits =
	(int)
	log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 0; i < nbits; i++)
	setBit(dnode, i, getBit(local_id, nbits - i - 1));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficShuffle()
{

    int nbits =
	(int)
	log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 0; i < nbits - 1; i++)
	setBit(dnode, i + 1, getBit(local_id, i));
    setBit(dnode, 0, getBit(local_id, nbits - 1));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

Packet ProcessingElement::trafficButterfly()
{

    int nbits = (int) log2ceil((double)
		 (GlobalParams::mesh_dim_x *
		  GlobalParams::mesh_dim_y));
    int dnode = 0;
    for (int i = 1; i < nbits - 1; i++)
	setBit(dnode, i, getBit(local_id, i));
    setBit(dnode, 0, getBit(local_id, nbits - 1));
    setBit(dnode, nbits - 1, getBit(local_id, 0));

    Packet p;
    p.src_id = local_id;
    p.dst_id = dnode;

    p.vc_id = randInt(0,GlobalParams::n_virtual_channels-1);
    p.timestamp = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
    p.size = p.flit_left = getRandomSize();

    return p;
}

void ProcessingElement::fixRanges(const Coord src,
				       Coord & dst)
{
    // Fix ranges
    if (dst.x < 0)
	dst.x = 0;
    if (dst.y < 0)
	dst.y = 0;
    if (dst.x >= GlobalParams::mesh_dim_x)
	dst.x = GlobalParams::mesh_dim_x - 1;
    if (dst.y >= GlobalParams::mesh_dim_y)
	dst.y = GlobalParams::mesh_dim_y - 1;
}

int ProcessingElement::getRandomSize()
{
    return randInt(GlobalParams::min_packet_size,
		   GlobalParams::max_packet_size);
}

unsigned int ProcessingElement::getQueueSize() const
{
    return packet_queue.size();
}

