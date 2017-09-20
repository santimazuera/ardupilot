// user defined variables
#include "APM_Config.h"
#include "Copter.h"
// example variables used in Wii camera testing - replace with your own
// variables

#ifdef USERHOOK_VARIABLES

class User_Sensors {
private:
    float GCS_RH[8];
    float GCS_IMET[8];

public:
    User_Sensors(){
        for(uint8_t i = 0; i<8; i++){
            GCS_IMET[i] = 0;
            GCS_RH[i] = 0;
        }
    }
    float get_RH(uint8_t i){ return GCS_RH[i];}
    float get_IMET(uint8_t i){ return GCS_IMET[i];}
    void set_RH(float val, uint8_t i){ GCS_RH[i] = val;}
    void set_IMET(float val, uint8_t i){ GCS_IMET[i] = val;}

};

extern User_Sensors user_sensor;

#endif  // USERHOOK_VARIABLES


