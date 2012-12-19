// Copyright 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.
/*
 * This file contains 3 implementations of the timer timer.
 * 3 implementations are provided:
 *  * one relies on boost::timer from boost >= 1.48, it is used preferentially
 *  * another relies on boost legacy timer, it is to be used with Visual Studio
 *  * another one relies on gettimeofday, for unix systems.
 */
#include "timer.hh"
#include <cassert>
#if BOOST_VERSION >= 104800
// new boost timer:
# include <boost/timer/timer.hpp>
#endif
// old boost timer:
# include <boost/timer.hpp>
#ifdef __GNUC__
# include <sys/time.h>
# include <cstddef>
#endif

namespace metapod
{
  namespace benchmark
  {
#if BOOST_VERSION >= 104800
    // simple wrapper
    class TimerBoostNew : public Timer
    {
    public:
      TimerBoostNew()
      {}

      void stop()
      {
        timer_.stop();
      }

      void start()
      {
        timer_.start();
      }

      void resume()
      {
        timer_.resume();
      }

      double void elapsed_wall_time_in_us()
      {
        return timer_.elapsed().wall * 1e3;
      }

    private:
      boost::timer::cpu_timer timer_;
    };
#endif

    // the old boost timer also works on posix systems but on these systems it
    // measures CPU time instead of wall clock time. So let build it,
    // but avoid using it.
    class TimerBoostLegacy : public Timer
    {
    public:
      TimerBoostLegacy():
        is_running_(true),
        time_(0.0)
      {}

      void stop()
      {
        if (is_running_)
        {
          time_ += elapsed_wall_time_in_us_();
          is_running_ = false;
        }
      }

      void resume()
      {
        if (!is_running_)
        {
          timer_.restart();
          is_running_ = true;
        }
      }

      void start()
      {
        time_ = 0.0;
        resume();
      }

      double elapsed_wall_time_in_us()
      {
        if (is_running_)
          return time_ + elapsed_wall_time_in_us_();
        else
          return time_;
      }

    private:
      double elapsed_wall_time_in_us_()
      {
        assert(is_running_);
        return timer_.elapsed() * 1e6;
      }
      bool is_running_;
      double time_;
      boost::timer timer_;
    };

#ifdef __GNUC__
    class TimerGetTimeOfDay : public Timer
    {
    public:
      TimerGetTimeOfDay():
        is_running_(false),
        start_(),
        time_(0.)
      {
        start();
      }

      void stop()
      {
        if (is_running_)
          time_ += elapsed_wall_time_in_us_();
        is_running_ = false;
      }

      void resume()
      {
        if (!is_running_)
        {
          ::gettimeofday(&start_, NULL);
          is_running_ = true;
        }
      }

      void start()
      {
        time_ = 0L;
        resume();
      }

      double elapsed_wall_time_in_us()
      {
        if (is_running_)
          return static_cast<double>(time_ + elapsed_wall_time_in_us_());
        else
          return static_cast<double>(time_);
      }

    private:
      long elapsed_wall_time_in_us_()
      {
        assert(is_running_);
        struct timeval stop;
        ::gettimeofday(&stop, NULL);
        return (stop.tv_sec - start_.tv_sec) * 1000000 +
               (stop.tv_usec - start_.tv_usec);
      }
      bool is_running_;
      struct timeval start_;
      long time_;
    };
#endif

    // factory function
    Timer* make_timer(void)
    {
#if BOOST_VERSION >= 104800
      return new TimerBoostNew();
#elif defined _MSC_VER
      return new TimerBoostLegacy();
#elif defined __GNUC__
      return new TimerGetTimeOfDay();
#else
# pragma error()
#endif
    }
  }
}
