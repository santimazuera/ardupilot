#include "Copter.h"
#include "UserParams_GCS.h"
#include <utility>

//Pixhawk I2C protocol parameters
AP_HAL::OwnPtr<AP_HAL::Device> dev_Temp[4], dev_RH[4];
extern const AP_HAL::HAL& hal;

//Sensors address
#define RH_IMET_ADDRESS_1       0x10
#define RH_IMET_ADDRESS_2       0x11
#define RH_IMET_ADDRESS_3       0x12
#define RH_IMET_ADDRESS_4		0x13

#define TEMP_IMET_ADDRESS_1       0x48
#define TEMP_IMET_ADDRESS_2       0x4A
#define TEMP_IMET_ADDRESS_3       0x4B
#define TEMP_IMET_ADDRESS_4       0x49


float coeff[4][3];
uint16_t config;

//Humidity sensor Params
float raw_H[4], rawRHt[4];
bool read_RH_sensor(uint8_t *data, uint32_t len, uint8_t i);
bool measure_RH_sensor(uint8_t i);
float RH_ant[4], RHt_ant[4];

//IMET sensor Params
int16_t rawT, rawV;
float resist, volt[4], curr[4];
bool flag;
bool config_IMET_sensor_TEMP(uint8_t i);
bool config_IMET_sensor_VOLT(uint8_t i);
bool read_IMET_sensor(uint8_t *data, uint32_t len, uint8_t i);
float temp_ant[4];
uint8_t Curr_data[4][2];
uint8_t Volt_data[4][2];

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up

    uint8_t RH_addr[4] = {RH_IMET_ADDRESS_1, RH_IMET_ADDRESS_2, RH_IMET_ADDRESS_3, RH_IMET_ADDRESS_4};
    uint8_t Temp_addr[4] = {TEMP_IMET_ADDRESS_1, TEMP_IMET_ADDRESS_2, TEMP_IMET_ADDRESS_3, TEMP_IMET_ADDRESS_4};

    config = ADS1015_REG_CONFIG_CQUE_NONE    | // Disable the comparator (default val)
             ADS1015_REG_CONFIG_CLAT_NONLAT  | // Non-latching (default val)
             ADS1015_REG_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
             ADS1015_REG_CONFIG_CMODE_TRAD   | // Traditional comparator (default val)
             ADS1015_REG_CONFIG_DR_1600SPS   | // 1600 samples per second (default)
             ADS1015_REG_CONFIG_MODE_SINGLE  | // Single-shot mode (default)
             ADS1015_REG_CONFIG_PGA_6_144V   | // Set PGA/voltage range
             ADS1015_REG_CONFIG_OS_SINGLE;     // Set start single-conversion bit

//    //PIXHAWK KIT SENSORS
//    //IMET temp number 48620:
//    coeff[0][0] = 1.01307391f * (float)pow(10, -3);
//    coeff[0][1] = 2.60659430f * (float)pow(10, -4);
//    coeff[0][2] = 1.52493329f * (float)pow(10, -7);

//    //IMET temp number 48626:
//    coeff[1][0] = 1.00565469f * (float)pow(10, -3);
//    coeff[1][1] = 2.62110037f * (float)pow(10, -4);
//    coeff[1][2] = 1.48522900f * (float)pow(10, -7);

//    //IMET temp number 45363:
//    coeff[2][0] = 9.93118592f * (float)pow(10, -4);
//    coeff[2][1] = 2.63743049f * (float)pow(10, -4);
//    coeff[2][2] = 1.47415476f * (float)pow(10, -7);

//    //IMET temp number 48622:
//    coeff[3][0] = 1.00880279f * (float)pow(10, -3);
//    coeff[3][1] = 2.61500024f * (float)pow(10, -4);
//    coeff[3][2] = 1.49421629f * (float)pow(10, -7);


//    //ORANGE COPTERSONDE SENSORS
    //IMET temp number 48623:
    coeff[0][0] = 1.00733068f * (float)pow(10, -3);
    coeff[0][1] = 2.62299300f * (float)pow(10, -4);
    coeff[0][2] = 1.48361439f * (float)pow(10, -7);

    //IMET temp number 48628:
    coeff[1][0] = 9.89775772f * (float)pow(10, -4);
    coeff[1][1] = 2.64144606f * (float)pow(10, -4);
    coeff[1][2] = 1.42274703f * (float)pow(10, -7);

    //IMET temp number 48627:
    coeff[2][0] = 1.00097308f * (float)pow(10, -3);
    coeff[2][1] = 2.62806129f * (float)pow(10, -4);
    coeff[2][2] = 1.46350112f * (float)pow(10, -7);

    //IMET temp number 45361:
    coeff[3][0] = 9.89681465f * (float)pow(10, -4);
    coeff[3][1] = 2.64132862f * (float)pow(10, -4);
    coeff[3][2] = 1.45628002f * (float)pow(10, -7);

    rawT=0; rawV=0;

    memset(raw_H,0,sizeof(raw_H));
    memset(rawRHt,0,sizeof(rawRHt));
    memset(temp_ant,0,sizeof(temp_ant));
    memset(RH_ant,0,sizeof(RH_ant));
    memset(RHt_ant,0,sizeof(RHt_ant));
    memset(volt,0,sizeof(volt));
    memset(curr,0,sizeof(curr));
    flag = true;
    resist = 0;


    for (uint8_t i=0; i<4; i++){
        //dev_RH[i] = std::move(hal.i2c_mgr->get_device(1, RH_addr[i]));
        dev_RH[i] = hal.i2c_mgr->get_device(1, RH_addr[i]);
        measure_RH_sensor(i);
        dev_RH[i]->get_semaphore()->give();
    }

    for (uint8_t i=0; i<4; i++){
        //dev_Temp[i] = std::move(hal.i2c_mgr->get_device(1, Temp_addr[i]));
        dev_Temp[i] = hal.i2c_mgr->get_device(1, Temp_addr[i]);
        config_IMET_sensor_TEMP(i);
        dev_Temp[i]->get_semaphore()->give();
    }

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    //float _last_read_ms = AP_HAL::millis();

    // put your 10Hz code here

    ///////////////////////////// READ TEMPERATURE SENSORS ///////////////////////////

    for (uint8_t i=0; i<4; i++) {
        if (dev_Temp[i]->get_semaphore()->take_nonblocking()) {
            if(flag) {
                if (!read_IMET_sensor(Curr_data[i], sizeof(Curr_data[i]), i)) {
                    memset(Curr_data[i],0,sizeof(Curr_data[i]));
                }
                config_IMET_sensor_VOLT(i);
                }
            else {
                if (!read_IMET_sensor(Volt_data[i], sizeof(Volt_data[i]), i)) {
                    memset(Volt_data[i],0,sizeof(Volt_data[i]));
                }

                curr[i] = (float)(((Curr_data[i][0] << 8) | Curr_data[i][1]));
                volt[i] = (float)(((Volt_data[i][0] << 8) | Volt_data[i][1]));
                resist = 64900.0 * (volt[i] / curr[i] - 1);
                volt[i] = curr[i] * 0.1875; //to store the voltage and send it
                curr[i] = 1.0 / (coeff[i][0] + coeff[i][1] * (float)log(resist) + coeff[i][2] * (float)pow((float)log((float)resist), 3)); //converts to temperature
                curr[i] -= 273.15; //covnerts to celsius
                if (curr[i] < -80 || curr[i] > 120 || (int)curr[i] == 0) {
                    curr[i] = temp_ant[i];
                }
                temp_ant[i] = curr[i];

                config_IMET_sensor_TEMP(i);
            }
            dev_Temp[i]->get_semaphore()->give();
        }
        else {
            return;
        }
    }

    flag = !flag;

    // Write sensors packet into the SD card
    if(flag) {
        struct log_IMET pkt_temp = {
            LOG_PACKET_HEADER_INIT(LOG_IMET_MSG),
            time_stamp             : (float)AP_HAL::millis(),
            temperature1           : curr[0],
            voltage1               : volt[0],
            temperature2           : curr[1],
            voltage2               : volt[1],
            temperature3           : curr[2],
            voltage3               : volt[2],
            temperature4           : curr[3],
            voltage4               : volt[3]
        };
        copter.DataFlash.WriteBlock(&pkt_temp, sizeof(pkt_temp));


//        mavlink_imet_temperature_humidity_group_t packet;
//        packet.time_boot_ms = AP_HAL::millis();
//        packet.temperature1 = rawRHt[0];
//        packet.humidity1 = raw_H[0];
//        packet.temperature2 = rawRHt[1];
//        packet.humidity2 = raw_H[1];
//        packet.temperature3 = rawRHt[2];
//        packet.humidity3 = raw_H[2];
//        packet.temperature4 = rawRHt[3];
//        packet.humidity4 = raw_H[3];
    
//        copter.send_imet_data(&packet);


        for (uint8_t i=0; i<4; i++){
            user_sensor.set_IMET(curr[i],i);
        }
        for (uint8_t i=4; i<8; i++){
            user_sensor.set_IMET(volt[i-4],i);
        }
    }

}
/////////////////// Sensors Functions //////////////////////

    /////////////Configure IMET temp sensor to read Temp/////////////////
    bool config_IMET_sensor_TEMP(uint8_t i){
        uint8_t Temp_cmd[] = {0x01, (uint8_t)((config | ADS1015_REG_CONFIG_MUX_SINGLE_0) >> 8), (uint8_t)((config | ADS1015_REG_CONFIG_MUX_SINGLE_0) & 0xFF)};

        if (!dev_Temp[i]->transfer(Temp_cmd, sizeof(Temp_cmd), nullptr, 0)) {
            return FALSE;
        }

        return TRUE;

    }

    /////////////Configure IMET temp sensor to read Volt/////////////////
    bool config_IMET_sensor_VOLT(uint8_t i){
        uint8_t Volt_cmd[] = {0x01, (uint8_t)((config | ADS1015_REG_CONFIG_MUX_SINGLE_1) >> 8), (uint8_t)((config | ADS1015_REG_CONFIG_MUX_SINGLE_1) & 0xFF)};

        if (!dev_Temp[i]->transfer(Volt_cmd, sizeof(Volt_cmd), nullptr, 0)) {
            return FALSE;
        }

        return TRUE;

    }

    /////////////Read from IMET sensor/////////////////
    bool read_IMET_sensor(uint8_t *data, uint32_t len, uint8_t i){

        uint8_t Temp_addr[] = {TEMP_IMET_ADDRESS_1, TEMP_IMET_ADDRESS_2, TEMP_IMET_ADDRESS_3, TEMP_IMET_ADDRESS_4};
        uint8_t Read_cmd[] = {0x00, uint8_t(Temp_addr[i] | 0x01)};

        //Read temperature
        if (!dev_Temp[i]->transfer(Read_cmd, 2, nullptr, 0)) {
            return FALSE;
        }

        hal.scheduler->delay(1);

        if (!dev_Temp[i]->transfer(nullptr, 0, data, len)) {
            return FALSE;
        }

        return TRUE;
    }
#endif

#ifdef USERHOOK_SLOWLOOP

void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here

    ///////////////////////////// READ RH SENSORS //////////////////////////////

    //float _last_read_ms = AP_HAL::millis();

    for (uint8_t i=0; i<4; i++) {

        if (dev_RH[i]->get_semaphore()->take_nonblocking()) {

            //collect data from sensors
            uint8_t RH_data[4][4];
            int16_t rawHumidity=0, rawRHtemp=0;

            //Read RH sensor
            //read_RH_sensor(RH_data[i], 4, i);

            ////////// Calculate Humidity and Temperature ////////////////
            if (read_RH_sensor(RH_data[i], 4, i)) {
                if ((RH_data[i][0] & 0x40) == 0) {
                    rawHumidity = (RH_data[i][0] << 8) | RH_data[i][1];
                    rawHumidity = rawHumidity & 0x3FFF;
                    raw_H[i] = (100.0 / ((float)pow(2,14) - 1)) * (float)rawHumidity;
                    RH_ant[i] = raw_H[i];

                    RH_data[i][3] = (RH_data[i][3] >> 2);
                    rawRHtemp = RH_data[i][2] << 6 | RH_data[i][3];
                    rawRHt[i] = (165.0 / ((float)pow(2,14) - 1)) * (float)rawRHtemp - 40;
                    RHt_ant[i] = rawRHt[i];

                    //send measure request to sensors
                    measure_RH_sensor(i);
                }
                else {
                    raw_H[i] = RH_ant[i];
                    rawRHt[i] = RHt_ant[i];
                    measure_RH_sensor(i);
                }
            }
            else {
                raw_H[i] = RH_ant[i];
                rawRHt[i] = RHt_ant[i];
                measure_RH_sensor(i);
            }

            dev_RH[i]->get_semaphore()->give();
        }
        else {
            return;
        }
    }

    // Write sensors packet into the SD card
    struct log_RH pkt_RH = {
        LOG_PACKET_HEADER_INIT(LOG_RH_MSG),
        time_stamp             : (float)AP_HAL::millis(), //- _last_read_ms),
        humidity1              : raw_H[0],
        RHtemp1                : rawRHt[0],
        humidity2              : raw_H[1],
        RHtemp2                : rawRHt[1],
        humidity3              : raw_H[2],
        RHtemp3                : rawRHt[2],
        humidity4              : raw_H[3],
        RHtemp4                : rawRHt[3]
    };
    copter.DataFlash.WriteBlock(&pkt_RH, sizeof(pkt_RH));
    for (uint8_t i=0; i<4; i++){
        user_sensor.set_RH(raw_H[i],i);
    }
    for (uint8_t i=4; i<8; i++){
        user_sensor.set_RH(rawRHt[i-4],i);
    }
}

/////////////////// Sensors Functions //////////////////////

bool measure_RH_sensor(uint8_t i) {
        uint8_t cmd = 0x00;
        if (!dev_RH[i]->transfer(&cmd, 1, nullptr, 0)) {
            return FALSE;
        }

        return TRUE;
    }

bool read_RH_sensor(uint8_t *data, uint32_t len, uint8_t i) {
    //Read sensors
    if (!dev_RH[i]->transfer(nullptr, 0, data, len)) {
        return FALSE;
    }

    return TRUE;
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif
