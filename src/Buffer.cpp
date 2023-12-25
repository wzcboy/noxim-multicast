/*
 * Noxim - the NoC Simulator
 *
 * (C) 2005-2018 by the University of Catania
 * For the complete list of authors refer to file ../doc/AUTHORS.txt
 * For the license applied to these sources refer to file ../doc/LICENSE.txt
 *
 * This file contains the implementation of the buffer
 */

#include "Buffer.h"
#include "Utils.h"

template <typename T>
Buffer<T>::Buffer()
{
  SetMaxBufferSize(GlobalParams::buffer_depth);
  max_occupancy = 0;
  hold_time = 0.0;
  last_event = 0.0;
  hold_time_sum = 0.0;

  previous_occupancy = 0;
  mean_occupancy = 0.0;
  true_buffer = true;
  full_cycles_counter = 0;
  last_front_flit_seq = NOT_VALID;
  deadlock_detected = false;
}

template <typename T>
void Buffer<T>::setLabel(string l)
{
    //cout << "\n BUFFER LABEL: " << l << endl;
    label = l;
}

template <typename T>
string Buffer<T>::getLabel() const
{
    return label;
}

template <typename T>
void Buffer<T>::Print()
{
//    queue<T> m = buffer;
//
//    string bstr = "";
//
//
//    char  t[] = "HBT";
//
//    cout << sc_time_stamp().to_double() / GlobalParams::clock_period_ps << "\t";
//    cout << label << " QUEUE *[";
//    while (!(m.empty()))
//    {
//	T f = m.front();
//	m.pop();
//	cout << bstr << t[f.flit_type] << f.sequence_no <<  "(" << f.dst_id << ") | ";
//    }
//    cout << "]*" << endl;
//    cout << endl;
}

template <typename T>
void Buffer<T>::deadlockCheck()
{
    // TOOD: add as parameter
//    int check_threshold = 50000;
//
//    if (IsEmpty()) return;
//
//    T f = buffer.front();
//    int seq = f.sequence_no;
//
//    if (last_front_flit_seq==seq)
//    {
//	full_cycles_counter++;
//    }
//    else
//    {
//	if (deadlock_detected)
//	{
//	    cout << " WRONG DEADLOCK detection, please increase the check_threshold " << endl;
//	    assert(false);
//	}
//	last_front_flit_seq = seq;
//	full_cycles_counter=0;
//    }
//
//    if (full_cycles_counter>check_threshold && !deadlock_detected)
//    {
//	double current_time = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
//	cout << "WARNING: DEADLOCK DETECTED at cycle " << current_time << " in buffer:  " << getLabel() << endl;
//	deadlock_detected = true;
//    }
}

template <typename T>
bool Buffer<T>::deadlockFree()
{
//    if (IsEmpty()) return true;
//
//    T f = buffer.front();
//
//    int seq = f.sequence_no;
//
//
//    if (last_front_flit_seq==seq)
//    {
//	full_cycles_counter++;
//    }
//    else
//    {
//	last_front_flit_seq = seq;
//	full_cycles_counter=0;
//    }
//
//    if (full_cycles_counter>50000)
//    {
//	return false;
//    }

    return true;

}

template <typename T>
void Buffer<T>::Disable()
{
  true_buffer = false;
}

template <typename T>
void Buffer<T>::SetMaxBufferSize(const unsigned int bms)
{
  assert(bms > 0);

  max_buffer_size = bms;
}

template <typename T>
unsigned int Buffer<T>::GetMaxBufferSize() const
{
  return max_buffer_size;
}

template <typename T>
bool Buffer<T>::IsFull() const
{
  return buffer.size() == max_buffer_size;
}

template <typename T>
bool Buffer<T>::IsEmpty() const
{
  return buffer.size() == 0;
}

template <typename T>
void Buffer<T>::Drop(const T & flit) const
{
  assert(false);
}

template <typename T>
void Buffer<T>::Empty() const
{
  assert(false);
}

template <typename T>
void Buffer<T>::Push(const T & flit)
{
  SaveOccupancyAndTime();

  if (IsFull())
    Drop(flit);
  else
    buffer.push_back(flit);
  
  UpdateMeanOccupancy();

  if (max_occupancy < buffer.size())
    max_occupancy = buffer.size();
}

template <typename T>
T Buffer<T>::Pop()
{
  T f;

  SaveOccupancyAndTime();

  if (IsEmpty())
    Empty();
  else {
    f = buffer.front();
    buffer.pop_front();
  }

  UpdateMeanOccupancy();

  return f;
}

template <typename T>
T Buffer<T>::Front() const
{
  T f;

  if (IsEmpty())
    Empty();
  else
    f = buffer.front();

  return f;
}

template <typename T>
void Buffer<T>::updateFront(const T& f)
{
    SaveOccupancyAndTime();
    buffer.front() = f;
    UpdateMeanOccupancy();
}

template <typename T>
unsigned int Buffer<T>::Size() const
{
  return buffer.size();
}

template <typename T>
unsigned int Buffer<T>::getCurrentFreeSlots() const
{
  return (GetMaxBufferSize() - Size());
}

template <typename T>
void Buffer<T>::SaveOccupancyAndTime()
{
  previous_occupancy = buffer.size();
  hold_time = (sc_time_stamp().to_double() / GlobalParams::clock_period_ps) - last_event;
  last_event = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
}

template <typename T>
void Buffer<T>::UpdateMeanOccupancy()
{
  double current_time = sc_time_stamp().to_double() / GlobalParams::clock_period_ps;
  if (current_time - GlobalParams::reset_time < GlobalParams::stats_warm_up_time)
    return;

  mean_occupancy = mean_occupancy * (hold_time_sum/(hold_time_sum+hold_time)) +
    (1.0/(hold_time_sum+hold_time)) * hold_time * buffer.size();

  hold_time_sum += hold_time;
}

template <typename T>
void Buffer<T>::ShowStats(std::ostream & out)
{
    out<< setiosflags(ios::fixed) << setprecision(2);

    if (true_buffer)
    out << "\t" << mean_occupancy << "\t" << max_occupancy;
  else
    out << "\t--\t--";
}

/************ Explicit Instantiate ***********/
template class Buffer<Flit>;
template class Buffer<MyPacket>;