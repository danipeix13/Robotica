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
         * Set the time value when the timer starts running
         */
        Timer(){
            begin = std::chrono::steady_clock::now();
        };

        /*
         * Use a thread that calls a function everytime the period conclude.
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
         *
         */
        void start(int p)
        {
			period.store(p);
			go.store(true);
        };

        /*
         *
         */
        void stop() { go.store(!go); };

        /*
         *
         */
        void setPeriod(int p) { period.store(p) ;};

        /*
         *
         */
        int getPeriod() { return period; };

        /*
         *
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
