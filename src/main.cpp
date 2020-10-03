#include <micro/debug/DebugLed.hpp>
#include <micro/hw/VL53L1X_DistanceSensor.hpp>
#include <micro/math/unit_utils.hpp>
#include <micro/panel/DistSensorPanelData.hpp>
#include <micro/utils/timer.hpp>

#include <cfg_board.hpp>

using namespace micro;

namespace {

hw::VL53L1X_DistanceSensor sensor(i2c_Sensor, 0x52);
PanelLink<DistSensorPanelInData, DistSensorPanelOutData> panelLink(panelLinkRole_t::Slave, uart_PanelLink);

meter_t distance;

void parseDistSensorPanelData(const DistSensorPanelInData& rxData) {
    UNUSED(rxData);
}

void fillDistSensorPanelData(DistSensorPanelOutData& txData) {
    txData.distance_mm = micro::isinf(distance) ? 0xffffu : static_cast<uint16_t>(static_cast<millimeter_t>(distance).get());
}

} // namespace

extern "C" void run(void) {
    sensor.initialize();

    DebugLed debugLed(gpio_Led);

    DistSensorPanelInData rxData;
    DistSensorPanelOutData txData;

    millisecond_t prevReadTime = getTime();
    bool isSensorOk = false;

    Timer sensorReadTimer(millisecond_t(20));

    while (true) {
        panelLink.update();

        if (sensorReadTimer.checkTimeout()) {
            const meter_t dist = sensor.readDistance();
            if (!micro::isinf(dist)) {
                distance = dist < centimeter_t(200) ? dist : micro::numeric_limits<meter_t>::infinity();
                isSensorOk = true;
                prevReadTime = getTime();

            } else if (getTime() - prevReadTime > millisecond_t(100)) {
                distance = meter_t(0);
                isSensorOk = false;
                prevReadTime = getTime();
            }
        }

        if (panelLink.readAvailable(rxData)) {
            parseDistSensorPanelData(rxData);
        }

        if (panelLink.shouldSend()) {
            fillDistSensorPanelData(txData);
            panelLink.send(txData);
        }

        debugLed.update(isSensorOk && panelLink.isConnected());
    }
}
