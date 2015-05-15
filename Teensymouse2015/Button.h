#ifndef BUTTON_H
#define BUTTON_H


class Button
{
public:
    Button();
    void init(int pin, bool activeState = true, bool pullup = false);
    void setPullup(bool on);
    void update();
    bool pressed();
    bool pressEdge();
    bool releaseEdge();
    
private:
    int pin;
    bool activeState;
    bool state;
    bool edge;
};

#endif // BUTTON_H
