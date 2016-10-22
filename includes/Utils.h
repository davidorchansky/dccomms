/*
 * Utils.h
 *
 *  Created on: 05/03/2015
 *      Author: diego
 */

#ifndef UTILS_H_
#define UTILS_H_
#include <cstdint>
#include <initializer_list>
#include <string>
#include <iostream>
#include <chrono>
#include <thread>

namespace dccomms {

#define LOG_DEBUG(msg) std::cerr << "DEBUG("+std::string(typeid(*this).name())+":"+std::string(__func__)+"): " + std::string(msg) << std::endl; std::cerr << std::flush;

class Utils {
public:
	Utils();
	virtual ~Utils();
	static bool IsBigEndian();
	static void IntSwitchEndian(void * b, uint32_t entero);
	static std::string BuildString(std::initializer_list<std::string> list );
	static void Debug(std::ostream &, std::string &);
	static void Sleep(int millis);

};

class Timer
{
public:
    Timer() : beg_(clock_::now()) {}
    void Reset() { beg_ = clock_::now(); }
    double Elapsed() const {
        return std::chrono::duration_cast<milis_>
            (clock_::now() - beg_).count(); }

private:
    typedef std::chrono::high_resolution_clock clock_;
    typedef std::chrono::duration<double, std::milli > milis_;
    std::chrono::time_point<clock_> beg_;
};

/*****************************/
/****** ServiceThread ********/
/*****************************/

template <class T> class ServiceThread
{
	//En C++11 una clase anidada puede acceder a los metodos de la "enclosing" class
public:
	ServiceThread(T *);
	~ServiceThread();
	bool IsRunning();
	void Start();
	void Stop();
	void Work();
	void (T::*_work)(void) = NULL;

	void SetWork(void (T::*f)(void))
	{
		_work = f;
	}

private:
	std::thread thread;
	bool mcontinue;
	bool terminated;
	bool started;
	T * comms;
};

template <class T>
ServiceThread<T>::ServiceThread(T * parent)
{
	mcontinue= true;
	terminated = false;
	started = false;
	comms = parent;
}

template <class T>
ServiceThread<T>::~ServiceThread()
{
	this->Stop();
}

template <class T>
void ServiceThread<T>::Start()
{
	thread = std::thread(&ServiceThread::Work, this);
	started = true;
}

template <class T>
void ServiceThread<T>::Stop()
{
	mcontinue = false;
}

template <class T>
bool ServiceThread<T>::IsRunning()
{
	return started && !terminated;
}

template <class T>
void ServiceThread<T>::Work()
{
	while(mcontinue)
	{
		(comms->*_work)();
	}
	terminated = true;
}

} /* namespace radiotransmission */

#endif /* UTILS_H_ */
