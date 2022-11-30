/*
 *
 *      Author: Eugene Shamaev
 */
#ifndef AP_UAVCAN_H_
#define AP_UAVCAN_H_

#include <uavcan/uavcan.hpp>

#include <AP_HAL/CAN.h>
#include <AP_HAL/Semaphores.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Param/AP_Param.h>

#include <AP_GPS/GPS_Backend.h>
#include <AP_Baro/AP_Baro_Backend.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_BattMonitor/AP_BattMonitor_Backend.h>

#include <uavcan/helpers/heap_based_pool_allocator.hpp>
#include <uavcan/equipment/indication/RGB565.hpp>

#ifndef UAVCAN_NODE_POOL_SIZE
#define UAVCAN_NODE_POOL_SIZE 128
#endif

#ifndef UAVCAN_NODE_POOL_BLOCK_SIZE
#define UAVCAN_NODE_POOL_BLOCK_SIZE 64
#endif

#ifndef UAVCAN_SRV_NUMBER
#define UAVCAN_SRV_NUMBER 18
#endif

#define AP_UAVCAN_MAX_LISTENERS 4
#define AP_UAVCAN_MAX_GPS_NODES 4
#define AP_UAVCAN_MAX_MAG_NODES 4
#define AP_UAVCAN_MAX_BARO_NODES 4
#define AP_UAVCAN_MAX_RANGEFINDER_NODES 4
#define AP_UAVCAN_MAX_BI_NUMBER 4

#define AP_UAVCAN_SW_VERS_MAJOR 1
#define AP_UAVCAN_SW_VERS_MINOR 0

#define AP_UAVCAN_HW_VERS_MAJOR 1
#define AP_UAVCAN_HW_VERS_MINOR 0

#define AP_UAVCAN_MAX_LED_DEVICES 4
#define AP_UAVCAN_LED_DELAY_MILLISECONDS 50

struct precland_uwb_range_s {
    bool valid {};
    uint32_t timestamp_ms;
    float range;
};

// fwd-declare callback classes
class TrafficReportCb;

extern struct precland_uwb_range_s precland_uwb_range;

/*
    Frontend Backend-Registry Binder: Whenever a message of said DataType_ from new node is received,
    the Callback will invoke registery to register the node as separate backend.
*/
#define UC_REGISTRY_BINDER(ClassName_, DataType_) \
	class ClassName_ : public AP_UAVCAN::RegistryBinder<DataType_> { \
        typedef void (*CN_Registry)(AP_UAVCAN*, uint8_t, const ClassName_&); \
	    public: \
	        ClassName_() : RegistryBinder() {} \
	        ClassName_(AP_UAVCAN* uc,  CN_Registry ffunc) : \
				RegistryBinder(uc, (Registry)ffunc) {} \
	}

class AP_UAVCAN {
public:
    AP_UAVCAN();
    ~AP_UAVCAN();

    static const struct AP_Param::GroupInfo var_info[];

    // Return uavcan from @iface or nullptr if it's not ready or doesn't exist
    static AP_UAVCAN *get_uavcan(uint8_t iface);

    // this function will register the listening class on a first free channel or on the specified channel
    // if preferred_channel = 0 then free channel will be searched for
    // if preferred_channel > 0 then listener will be added to specific channel
    // return value is the number of assigned channel or 0 if fault
    // channel numbering starts from 1
    uint8_t register_gps_listener(AP_GPS_Backend* new_listener, uint8_t preferred_channel);

    uint8_t register_gps_listener_to_node(AP_GPS_Backend* new_listener, uint8_t node);

    uint8_t find_gps_without_listener(void);

    // Removes specified listener from all nodes
    void remove_gps_listener(AP_GPS_Backend* rem_listener);

    // Returns pointer to GPS state connected with specified node.
    // If node is not found and there are free space, locate a new one
    AP_GPS::GPS_State *find_gps_node(uint8_t node);

    // Updates all listeners of specified node
    void update_gps_state(uint8_t node);

    struct Baro_Info {
        float pressure;
        float pressure_variance;
        float temperature;
        float temperature_variance;
    };

    uint8_t register_baro_listener(AP_Baro_Backend* new_listener, uint8_t preferred_channel);
    uint8_t register_baro_listener_to_node(AP_Baro_Backend* new_listener, uint8_t node);
    void remove_baro_listener(AP_Baro_Backend* rem_listener);
    Baro_Info *find_baro_node(uint8_t node);
    uint8_t find_smallest_free_baro_node();
    void update_baro_state(uint8_t node);

    struct Mag_Info {
        Vector3f mag_vector;
    };

    uint8_t register_mag_listener(AP_Compass_Backend* new_listener, uint8_t preferred_channel);
    void remove_mag_listener(AP_Compass_Backend* rem_listener);
    Mag_Info *find_mag_node(uint8_t node, uint8_t sensor_id);
    uint8_t find_smallest_free_mag_node();
    uint8_t register_mag_listener_to_node(AP_Compass_Backend* new_listener, uint8_t node);
    void update_mag_state(uint8_t node, uint8_t sensor_id);

    struct BatteryInfo_Info {
        float temperature;
        float voltage;
        float current;
        float remaining_capacity_wh;
        float full_charge_capacity_wh;
        uint8_t status_flags;
    };

    uint8_t register_BM_bi_listener_to_id(AP_BattMonitor_Backend* new_listener, uint8_t id);
    void remove_BM_bi_listener(AP_BattMonitor_Backend* rem_listener);
    BatteryInfo_Info *find_bi_id(uint8_t id);
    uint8_t find_smallest_free_bi_id();
    void update_bi_state(uint8_t id);

    struct Rangefinder_Info {
        float distance;
        uint8_t status;
    };

    uint8_t register_rangefinder_listener(AP_RangeFinder_Backend* new_listener, uint8_t preferred_channel);
    uint8_t register_rangefinder_listener_to_node(AP_RangeFinder_Backend* new_listener, uint8_t node);
    void remove_rangefinder_listener(AP_RangeFinder_Backend* rem_listener);
    Rangefinder_Info *find_rangefinder_node(uint8_t node);
    uint8_t find_smallest_free_rangefinder_node();
    void update_rangefinder_state(uint8_t node);
    
    // synchronization for RC output
    void SRV_sem_take();
    void SRV_sem_give();
    
    // synchronization for LED output
    bool led_out_sem_take();
    void led_out_sem_give();
    void led_out_send();

    // output from do_cyclic
    void SRV_send_servos();
    void SRV_send_esc();

    // send RTCMStream packets
    void send_RTCMStream(const uint8_t *data, uint32_t len);

    template <typename DataType_>
    class RegistryBinder {
    protected:
        typedef void (*Registry)(AP_UAVCAN* _ap_uavcan, uint8_t _node_id, const RegistryBinder& _cb);
        AP_UAVCAN* _uc;
        Registry _ffunc;

    public:
        RegistryBinder() :
        	_uc(),
            _ffunc(),
            msg() {}

        RegistryBinder(AP_UAVCAN* uc, Registry ffunc) :
            _uc(uc),
            _ffunc(ffunc),
            msg(nullptr) {}

        void operator()(const uavcan::ReceivedDataStructure<DataType_>& _msg) {
            msg = &_msg;
            _ffunc(_uc, _msg.getSrcNodeID().get(), *this);
        }

        const uavcan::ReceivedDataStructure<DataType_> *msg;
    };

private:
    // ------------------------- GPS
    // 255 - means free node
    uint8_t _gps_nodes[AP_UAVCAN_MAX_GPS_NODES];
    // Counter of how many listeners are connected to this source
    uint8_t _gps_node_taken[AP_UAVCAN_MAX_GPS_NODES];
    // GPS data of the sources
    AP_GPS::GPS_State _gps_node_state[AP_UAVCAN_MAX_GPS_NODES];

    // 255 - means no connection
    uint8_t _gps_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    // Listeners with callbacks to be updated
    AP_GPS_Backend* _gps_listeners[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- BARO
    uint8_t _baro_nodes[AP_UAVCAN_MAX_BARO_NODES];
    uint8_t _baro_node_taken[AP_UAVCAN_MAX_BARO_NODES];
    Baro_Info _baro_node_state[AP_UAVCAN_MAX_BARO_NODES];
    uint8_t _baro_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    AP_Baro_Backend* _baro_listeners[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- MAG
    uint8_t _mag_nodes[AP_UAVCAN_MAX_MAG_NODES];
    uint8_t _mag_node_taken[AP_UAVCAN_MAX_MAG_NODES];
    Mag_Info _mag_node_state[AP_UAVCAN_MAX_MAG_NODES];
    uint8_t _mag_node_max_sensorid_count[AP_UAVCAN_MAX_MAG_NODES];
    uint8_t _mag_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    AP_Compass_Backend* _mag_listeners[AP_UAVCAN_MAX_LISTENERS];
    uint8_t _mag_listener_sensor_ids[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- BatteryInfo
    uint16_t _bi_id[AP_UAVCAN_MAX_BI_NUMBER];
    uint16_t _bi_id_taken[AP_UAVCAN_MAX_BI_NUMBER];
    BatteryInfo_Info _bi_id_state[AP_UAVCAN_MAX_BI_NUMBER];
    uint16_t _bi_BM_listener_to_id[AP_UAVCAN_MAX_LISTENERS];
    AP_BattMonitor_Backend* _bi_BM_listeners[AP_UAVCAN_MAX_LISTENERS];

    // ------------------------- RANGEFINDER
    uint8_t _rangefinder_nodes[AP_UAVCAN_MAX_RANGEFINDER_NODES];
    uint8_t _rangefinder_node_taken[AP_UAVCAN_MAX_RANGEFINDER_NODES];
    Rangefinder_Info _rangefinder_node_state[AP_UAVCAN_MAX_RANGEFINDER_NODES];
    uint8_t _rangefinder_listener_to_node[AP_UAVCAN_MAX_LISTENERS];
    AP_RangeFinder_Backend* _rangefinder_listeners[AP_UAVCAN_MAX_LISTENERS];
    
    struct {
        uint16_t pulse;
        uint16_t safety_pulse;
        uint16_t failsafe_pulse;
        bool esc_pending;
        bool servo_pending;
    } _SRV_conf[UAVCAN_SRV_NUMBER];

    bool _initialized;
    uint8_t _SRV_armed;
    uint8_t _SRV_safety;
    uint32_t _SRV_last_send_us;

    typedef struct {
        bool enabled;
        uint8_t led_index;
        uavcan::equipment::indication::RGB565 rgb565_color;
    } _led_device;

    struct {
        bool broadcast_enabled;
        _led_device devices[AP_UAVCAN_MAX_LED_DEVICES];
        uint8_t devices_count;
        uint64_t last_update;
    } _led_conf;

    AP_HAL::Semaphore *SRV_sem;
    AP_HAL::Semaphore *_led_out_sem;

    class SystemClock: public uavcan::ISystemClock, uavcan::Noncopyable {
        SystemClock()
        {
        }

        uavcan::UtcDuration utc_adjustment;
        virtual void adjustUtc(uavcan::UtcDuration adjustment)
        {
            utc_adjustment = adjustment;
        }

    public:
        virtual uavcan::MonotonicTime getMonotonic() const
        {
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            return uavcan::MonotonicTime::fromUSec(usec);
        }
        virtual uavcan::UtcTime getUtc() const
        {
            uavcan::UtcTime utc;
            uavcan::uint64_t usec = 0;
            usec = AP_HAL::micros64();
            utc.fromUSec(usec);
            utc += utc_adjustment;
            return utc;
        }

        static SystemClock& instance()
        {
            static SystemClock inst;
            return inst;
        }
    };

    uavcan::Node<0> *_node = nullptr;

    uavcan::ISystemClock& get_system_clock();
    uavcan::ICanDriver* get_can_driver();
    uavcan::Node<0>* get_node();

    // This will be needed to implement if UAVCAN is used with multithreading
    // Such cases will be firmware update, etc.
    class RaiiSynchronizer {
    public:
        RaiiSynchronizer()
        {
        }

        ~RaiiSynchronizer()
        {
        }
    };

    uavcan::PoolAllocator<UAVCAN_NODE_POOL_SIZE*UAVCAN_NODE_POOL_BLOCK_SIZE, UAVCAN_NODE_POOL_BLOCK_SIZE, AP_UAVCAN::RaiiSynchronizer> _node_allocator;

    // send GNSS injection
    void rtcm_stream_send();
    AP_Int8 _uavcan_node;
    AP_Int32 _servo_bm;
    AP_Int32 _esc_bm;
    AP_Int16 _servo_rate_hz;

    uint8_t _uavcan_i;

    AP_HAL::CANManager* _parent_can_mgr;

public:
    void do_cyclic(void);
    bool try_init(void);

    void SRV_set_safety_pwm(uint32_t chmask, uint16_t pulse_len);
    void SRV_set_failsafe_pwm(uint32_t chmask, uint16_t pulse_len);
    void SRV_force_safety_on(void);
    void SRV_force_safety_off(void);
    void SRV_arm_actuators(bool arm);
    void SRV_write(uint16_t pulse_len, uint8_t ch);
    void SRV_push_servos(void);
    bool led_write(uint8_t led_index, uint8_t red, uint8_t green, uint8_t blue);

    void set_parent_can_mgr(AP_HAL::CANManager* parent_can_mgr)
    {
        _parent_can_mgr = parent_can_mgr;
    }
    // GNSS RTCM injection
    struct {
        AP_HAL::Semaphore *sem;
        uint32_t last_send_ms;
        ByteBuffer *buf;
    } _rtcm_stream;
    static void handle_traffic_report(AP_UAVCAN* ap_uavcan, uint8_t node_id, const TrafficReportCb &cb);
};

#endif /* AP_UAVCAN_H_ */
