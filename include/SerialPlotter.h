/**
 * @file SerialPlotter.h
 * @author Ilia Moroz (iliamorozim@gmail.com)
 * @brief SerialPlotter class for conveniently plotting data through Serial port
 * @version 1.0
 * @date 2025-07-06
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#ifndef FROSTER_SERIALPLOTTER_H
#define FROSTER_SERIALPLOTTER_H
#include <Arduino.h>


#ifndef FROSTER_TIMER_H
#define FROSTER_TIMER_H

class Timer
{
public:
  Timer(uint16_t period = 1000)
  {
    _period = period;
    _timer = millis();
  }
  bool isReady()
  {
    if (millis() - _timer >= _period)
    {
      _timer = millis();
      return true;
    }
    return false;
  }
  void setPeriod(uint16_t newPeriod)
  {
    _period = newPeriod;
  }

private:
  uint16_t _period;
  uint32_t _timer;
};

#endif

#define DEFAULT_TIMER_PERIOD 100

template <typename T = float>
class BaseParameter
{
public:
    virtual ~BaseParameter() = default;
    virtual T get_value() = 0;
    virtual const char *get_name() = 0;
};

template <typename T = float>
class DirectParameter : public BaseParameter<T>
{
public:
    DirectParameter(const char *name, T *value) : _name(name), _value(value)
    {
    }

    const char *get_name() override
    {
        return _name;
    }

    T get_value() override
    {
        return *_value;
    }

private:
    const char *_name;
    T *_value;
};

template <typename T = float>
class FuncParameter : public BaseParameter<T>
{
public:
    FuncParameter(const char *name, T (*func)()) : _name(name), _func(func)
    {
    }

    const char *get_name() override
    {
        return _name;
    }

    T get_value() override
    {
        return static_cast<T>((*_func)());
    }

private:
    const char *_name;
    T (*_func)();
};

class BasePlot
{
public:
    virtual ~BasePlot() = default;
    virtual String generate_command() = 0;
};

template <uint8_t parameters_number, typename T = float>
class Plot : public BasePlot
{
public:
    Plot(const char *name)
    {
        _name = name;
    };

    ~Plot() = default;

    void attach_parameter(const char *name, T *value)
    {
        if (_parameter_count >= parameters_number)
            return;
        DirectParameter<T> *new_parameter = new DirectParameter<T>(name, value);
        _parameters[_parameter_count++] = new_parameter;
    };

    void attach_parameter(const char *name, T (*func)())
    {
        if (_parameter_count >= parameters_number)
        {
            Serial.println(F("Too many parameters attached to the plot"));
            return;
        }

        FuncParameter<T> *new_parameter = new FuncParameter<T>(name, func);
        _parameters[_parameter_count++] = new_parameter;
    };

    String generate_command()
    {
        String command = _name;
        command += F("(");
        command.reserve(64); // TODO: Remove magic numbers
        for (uint8_t i = 0; i < parameters_number; i++)
        {
            command += String(_parameters[i]->get_name());
            command += F(":");
            command += String(_parameters[i]->get_value());
            if (i < parameters_number - 1)
            {
                command += F(",");
            }
        }
        command += F(")");
        return command;
    };

    uint8_t get_parameters_number()
    {
        return parameters_number;
    }

private:
    BaseParameter<T> *_parameters[parameters_number];
    uint8_t _parameter_count = 0;
    const char *_name;
};

template <uint8_t plots_number>
class SerialPlotter
{
public:
    SerialPlotter() : _timer(DEFAULT_TIMER_PERIOD) {};

    ~SerialPlotter()
    {
        for (uint8_t i = 0; i < _plots_count; i++)
        {
            delete static_cast<BasePlot *>(_plots[i]);
        }
    };

    template <uint8_t parameters_number, typename T = float>
    Plot<parameters_number, T> &add_plot(const char *name)
    {
        if (_plots_count >= plots_number)
        {
            Serial.println(F("Too many plots added to one plotter"));
        }
        static_assert(parameters_number > 0, "Plot must have at least one parameter");
        Plot<parameters_number, T> *new_plot = new Plot<parameters_number, T>(name);
        _plots[_plots_count] = new_plot;
        return *static_cast<Plot<parameters_number, T> *>(_plots[_plots_count++]);
    };

    String plot()
    {
        if (_timer.isReady())
        {
            String command = F("{");
            command.reserve(64); // TODO: Add a calaulation of this number
            for (uint8_t i = 0; i < plots_number; i++)
            {
                command += static_cast<BasePlot *>(_plots[i])->generate_command();
            }
            command += F("}");

            Serial.print(command);

            return command;
        }
        return String("");
    }

    void set_period(uint32_t &period)
    {
        _timer.setPeriod(period);
    }

private:
    void *_plots[plots_number];
    uint8_t _plots_count = 0;
    Timer _timer;
};

#endif