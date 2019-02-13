/*----------------------------------------------------------------------------------*/

//Header file for LED.cpp

/*----------------------------------------------------------------------------------*/

#pragma once



class LED {

    public:

        static LED* GetInstance();

    private:

        static LED* s_instance;
        LED();

        

};