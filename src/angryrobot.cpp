#include <EEPROM.h>

#ifdef UNITTEST
#include <iostream>
#define PRINT(arg) std::cout<<arg<<std::endl
#define ERROR(arg) std::cerr<<arg<<std::cerr
#else
#include "Arduino.h"
#define PRINT(arg) Serial.println(arg)
#define ERROR(arg) Serial.println(arg)

// distance detector
constexpr auto TRIGGER_PIN = 16; 
constexpr auto ECHO_PIN    = 17;

// internal led
//constexpr auto LED_PIN = 13; // conflict with motor shield

#endif

template <class T>
class Sensors {
  private:
    T sound; // range [0:255]
    
  public:
    explicit Sensors() : sound(0) {}
    
    inline T readSound() noexcept {
        return 0;
    }
    
    inline long readObstacle() noexcept {
        digitalWrite(TRIGGER_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN, LOW);
        return pulseIn(ECHO_PIN, HIGH) / 58;
    }
    
    inline T getSound() noexcept {
        return 0; 
    }
};

class Motor {
  private:
    size_t forwardDirectionPin;
    size_t brakePin;
    size_t speedPin;
    
  public:
    Motor(size_t fd, size_t bk, size_t sp) : forwardDirectionPin(fd), brakePin(bk), speedPin(sp) {
        pinMode(forwardDirectionPin,OUTPUT);     // initates motor channel pin
        pinMode(brakePin,OUTPUT);                // initiate Brake Channel pin
    }
    
    void forward(size_t speed) {
        digitalWrite(forwardDirectionPin, HIGH); //Establishes forward direction of Channel
        digitalWrite(brakePin, LOW);             //Disengage the Brake for Channel
        analogWrite(speedPin, speed);            //Spins the motor on Channel at speed
    }
    
    void backward(size_t speed) {
        digitalWrite(forwardDirectionPin, LOW);  //Establishes backward direction of Channel
        digitalWrite(brakePin, LOW);             //Disengage the Brake for Channel
        analogWrite(speedPin, speed);            //Spins the motor on Channel at speed     
    }

    void stop() {
        digitalWrite(brakePin, HIGH);            //Disengage the Brake for Channel
    }
};

template <class T>
class Move {
  private:

    Motor* channelA;
    Motor* channelB;

  public:
    Move() : channelA(new Motor(12,9,3)), channelB(new Motor(13,8,11)) {}

    inline void brake() noexcept {
        channelA->stop();
        channelB->stop();
    }
    
    inline void turnLeft(size_t speed)  noexcept {
        channelA->stop();
        channelB->forward(speed);
    }
    
    inline void turnRight(size_t speed) noexcept {
        channelA->forward(speed); 
        channelB->stop();
    }
    
    inline void halfTurn(size_t speed)  noexcept {
        channelA->forward(speed);
        channelB->backward(speed);
    }
    
    inline void straight(size_t speed)  noexcept {
        channelA->forward(speed);
        channelB->forward(speed);
    }
    inline void randomTurn() noexcept {
        PRINT(random(-100,100));
    }
};

class Manager {
  using SType = size_t;
  using MType = size_t;
  
  private:
    Sensors<SType> m_s;
    Move<MType>    m_m;
    
  public:
    explicit Manager() : m_s(), m_m() { PRINT("Manager created"); }
    inline Move<MType>*    getMove()   { return &m_m; }
    inline Sensors<SType>* getSensor() { return &m_s; }
};

enum class Status : size_t {good = 1, bad = 0, unknown = 100};

class Robot {
  private:
    Manager* mgr;
    static constexpr size_t threshold = 0;

  public:
    Status status = Status::unknown;

    explicit Robot() : mgr(nullptr) {  // def ctor
        PRINT("Robot created"); 
        this->start();
    }

    inline void start() noexcept {
        PRINT("start");
        status = Status::good;
        mgr = new Manager();
        if (mgr==nullptr) {
          ERROR("could not create Manager");
          this->stop();
        }
    }

    inline void stop() noexcept {
        status = Status::bad;
    }

    auto readDistance() noexcept -> decltype(mgr->getSensor()->readObstacle()) {
        return mgr->getSensor()->readObstacle();
    }

    inline bool detectSound() noexcept {
        return (mgr->getSensor()->getSound() > this->threshold) ;
    }

    inline void moveToSource() noexcept {
        mgr->getMove()->straight(100);
    }

    inline void straight() noexcept {
        mgr->getMove()->straight(150);
    }

    inline void brake() noexcept {
        PRINT("BRAKE");
        mgr->getMove()->brake();
    }

    inline void turnLeft() noexcept {
        mgr->getMove()->turnLeft(100);
    }

    inline void randomLeft() noexcept {
        mgr->getMove()->turnLeft(random(100,200));
    }

    // Rapidly-exploring random tree algorithm (RRT)?
};


Robot* robot;
unsigned long timer;
#define INTERVAL 1000

void startInterruptionTimer() {
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 16e6 / 50 / 1024 - 1; // 16Mhz / freq / prescaler - 1
    //OCR1A = 0x3D08; // set compare match register to desired timer count
    TCCR1B |= (1 << WGM12); // Mode 4, CTC on OCR1A
    TIMSK1 |= (1 << OCIE1A); // Set interrupt on compare match
    TCCR1B |= (1 << CS12) | (1 << CS10); // set prescaler to 1024 and start the timer
    interrupts();
}

void setup() {
    Serial.begin(9600);
    while (!Serial); 
    PRINT("setup");
    robot = new Robot();
    timer = millis();
    
    //pinMode(LED_PIN, OUTPUT);
    pinMode(TRIGGER_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    startInterruptionTimer();
}



void loop() {
    if(Status::good != robot->status)
        return;
    
    //if(millis()-timer>INTERVAL)
    //{
    //    timer += INTERVAL;
        auto dist = robot->readDistance();
        if (dist < 20 && dist > 0 ) {
            robot->brake();
            delay(1000);
            //robot->turnLeft();
            robot->randomLeft();
            delay(1000);
            robot->brake();
            delay(1000);
        }
        else {
            robot->straight();
        }
    //}
}

ISR (TIMER1_COMPA_vect) 
{
    //digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
}

#ifdef UNITTEST
int main() {
    setup();
    for(;;) {
        loop();
    }
}
#endif