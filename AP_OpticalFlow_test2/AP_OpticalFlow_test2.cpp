/*
 *  Example of AP_OpticalFlow library.
 *  Code by ChoYG. whdlgp@gmail.com
 */

#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_NavEKF2/AP_NavEKF2.h>
#include <AP_NavEKF/AP_NavEKF.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_RangeFinder/AP_RangeFinder.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class DummyVehicle {
public:
    void setup(void);
    void loop(void);
private:
    //sensor & filter declaration
    AP_GPS gps;
    AP_Baro barometer;
    Compass compass;
    AP_InertialSensor ins;
    AP_SerialManager serial_manager;
    RangeFinder sonar { serial_manager };
    AP_AHRS_NavEKF ahrs { ins, barometer, gps, sonar, EKF, EKF2,
            AP_AHRS_NavEKF::FLAG_ALWAYS_USE_EKF };
    NavEKF EKF { &ahrs, barometer, sonar };
    NavEKF2 EKF2 { &ahrs, barometer, sonar };

    OpticalFlow optflow { ahrs };

    AP_Scheduler scheduler;
    static const AP_Scheduler::Task scheduler_tasks[];

    uint8_t flowQuality;
    Vector2f flowRate;
    Vector2f bodyRate;

    void ins_update(void); //for scheduler timing
    void update_optical_flow(void);
    void one_hz_print(void);
};

//for scheduler timing
void DummyVehicle::ins_update(void) {
    ins.update();
}

void DummyVehicle::update_optical_flow(void) {
    static uint32_t last_of_update = 0;

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        flowQuality = optflow.quality();
        flowRate = optflow.flowRate();
        bodyRate = optflow.bodyRate();
    }
}

void DummyVehicle::one_hz_print(void) {
    hal.console->printf("Quality : %d\n", this->flowQuality);
    hal.console->printf("flowRate.x : %f\n", this->flowRate.x);
    hal.console->printf("flowRate.y : %f\n", this->flowRate.y);
    hal.console->printf("bodyRate.x : %f\n", this->bodyRate.x);
    hal.console->printf("bodyRate.y : %f\n", this->bodyRate.y);
}

static DummyVehicle vehicle;

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) SCHED_TASK_CLASS(DummyVehicle, &vehicle, func, _interval_ticks, _max_time_micros)

const AP_Scheduler::Task DummyVehicle::scheduler_tasks[] = {
        SCHED_TASK(ins_update           ,50     ,1000   ), //for scheduler timing
        SCHED_TASK(update_optical_flow  ,200    ,160    ),
        SCHED_TASK(one_hz_print         ,1      ,1000   ),
};

void DummyVehicle::setup(void) {
    AP_BoardConfig { }.init();

    //setup optical flow sensor
    ahrs.set_optflow(&optflow);
    //if not enabled
    if (!optflow.enabled()) {
        hal.console->println("optical flow not enabled");
    }

    // initialize optical flow sensor
    optflow.init();

    hal.console->println("OpticalFlow library test");

    //setup loop timing
    ins.init(scheduler.get_loop_rate_hz());

    // initialize the scheduler
    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void DummyVehicle::loop(void) {
    // wait for an INS sample
    ins.wait_for_sample();

    // tell the scheduler one tick has passed
    scheduler.tick();

    // run all tasks that fit in 20ms
    scheduler.run(20000);
}

void setup() {
    vehicle.setup();
}

void loop() {
    vehicle.loop();
}

AP_HAL_MAIN();
