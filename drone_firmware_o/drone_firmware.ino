// UTIL

namespace Util 
{
  enum LogSeverity
  {
    LSFlood,
    LSInfo,
    LSWarning,
    LSError,
    LSCritical
  };

  void init()
  {
    Serial.begin(115200);
  }

  void log(const char * msg, LogSeverity sev = LSInfo)
  {
    Serial.print(msg);
    Serial.print("\n");
  }
  
}

#define BIT(x) (1 << (x))

// SCHEDULING

#include <string>

class CETask
{
public:
  virtual void tick() = 0;
};

template <int NUM_TASKS, int NUM_COMPARTMENTS, int COMPART_US>
class CEScheduler
{
public:
  typedef unsigned char task_mask_t;
  
  CEScheduler()
  {
    memset(_tasks, 0, sizeof(_tasks));
    memset(_schedule, 0, sizeof(_schedule));
  }

  void setTask(CETask * t, int index)
  {
    _tasks[index] = t;
  }

  void addTaskToCompartment(int taskI, int compartmentI)
  {
    _schedule[compartmentI] |= 1 << taskI;
  }
  
  void setCompartment(task_mask_t mask, int compartmentI)
  {
    _schedule[compartmentI] = mask;
  }

  void exec()
  {
    int currCompartment = 0;
    
    unsigned long lastTimestamp = micros();
    unsigned long currentTimestamp = lastTimestamp;
    for(;;)//ever
    {
      while( currentTimestamp - lastTimestamp < COMPART_US )
      {
        currentTimestamp = micros();
      }

      lastTimestamp = currentTimestamp - (currentTimestamp - lastTimestamp - COMPART_US);

      // this is a tick
      // now we run the tasks
      for(int i = 0; i < NUM_TASKS; i++)
      {
        if(!_tasks[i])
          continue;

        if(_schedule[currCompartment] & BIT(i))
          _tasks[i]->tick();
        
      }

      currCompartment = (currCompartment+1) % NUM_COMPARTMENTS;
    } // forever
  }
  
private:
  

  CETask * _tasks[NUM_TASKS];
  task_mask_t _schedule[NUM_COMPARTMENTS];
  
};

// BLACKBOARD

class Blackboard
{
public:
  static Blackboard instance;

  int x;
  bool y;
};

Blackboard Blackboard::instance;
#define BB Blackboard::instance

// MAIN

class SysTask: public CETask
{
public:
  void tick()
  {
    Util::log("Hello from system task");
    BB.y++;
  }
};

class NavTask: public CETask
{
public:
  void tick()
  {
    Util::log("Hello from nav task");
    BB.x++;
  }
  
};

CEScheduler<2,2,500000> scheduler;
SysTask sysTask;
NavTask navTask;

void createScheduleTable()
{
  scheduler.setTask(&sysTask, 0); // task 0 is sys
  scheduler.setTask(&navTask, 1); // task 1 is nav

  scheduler.setCompartment( BIT(0) | BIT(1), 0 ); // first compartment: sys, nav
  scheduler.setCompartment( BIT(0)         , 1 ); // second compartment: sys
}

void setup() 
{

  Util::init();

  createScheduleTable();
  
}

void loop() {
  scheduler.exec();
}
