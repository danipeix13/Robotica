#ifndef TIMER_H
#define TIMER_H

#include <thread>
#include <chrono>
#include <functional>
#include <future>
#include <cstdio>
#include <iostream>

//Manual Timer
class Timer
{
    public:

        /*
         * Sets the time value when the timer starts running
         */
        Timer(){
            begin = std::chrono::steady_clock::now();
        };

        /*
         * Uses a thread with a lambda that calls a function f everytime that the period conclude.
         */
        template <class callable>
        void connect(callable&& f)
        {
			std::thread([=]() 
            {
                while(true)
                {
					if(go.load())//Timer is running
						std::invoke(f);
                    std::this_thread::sleep_for(std::chrono::milliseconds(period.load()));
                }
            }).detach();
        };

        /*
         * Starts the timer with a period p
         */
        void start(int p)
        {
			period.store(p);
			go.store(true);
        };

        /*
         * Stops the timer
         */
        void stop() { go.store(!go); };

        /*
         * Changes the timer period to p
         */
        void setPeriod(int p) { period.store(p) ;};

        /*
         * Returns the period value
         */
        int getPeriod() { return period; };

        /*
         * Prints the elapsed time since the timer is initiated
         */
        void elapsedTime(){
            end = std::chrono::steady_clock::now();
            std::cout << "Elapsed time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms" << std::endl;
        };

        
    private:
        std::atomic_bool go = false;
		std::atomic_int period = 0;
        std::chrono::steady_clock::time_point begin, end;
};

#endif // TIMER_H
