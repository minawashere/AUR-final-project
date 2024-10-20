#ifndef IRSENSOR_H
#define IRSENSOR_H



class IRsensor {
private:
    int pin;
public:
    explicit IRsensor(int pin);
    bool read();
};


#endif //IRSENSOR_H
