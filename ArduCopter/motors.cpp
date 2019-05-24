#include "Copter.h"

#define ARM_DELAY               20  // called at 10hz so 2 seconds
#define DISARM_DELAY            20  // called at 10hz so 2 seconds
#define AUTO_TRIM_DELAY         100 // called at 10hz so 10 seconds
#define LOST_VEHICLE_DELAY      10  // called at 10hz so 1 second

static uint32_t auto_disarm_begin;

/*
  playback log captured from high vibration flight
 */
const Copter::mot4 Copter::playback[] = {
    {1514,1522,1521,1565},
    {1545,1481,1473,1607},
    {1514,1500,1468,1606},
    {1520,1490,1483,1597},
    {1534,1466,1505,1578},
    {1516,1509,1520,1561},
    {1553,1550,1476,1549},
    {1569,1571,1474,1517},
    {1574,1550,1469,1518},
    {1541,1564,1501,1536},
    {1537,1529,1496,1556},
    {1515,1523,1497,1577},
    {1524,1494,1502,1577},
    {1519,1499,1492,1590},
    {1509,1502,1487,1590},
    {1502,1485,1483,1568},
    {1488,1484,1426,1580},
    {1516,1471,1455,1555},
    {1506,1483,1443,1548},
    {1492,1501,1432,1556},
    {1503,1505,1469,1539},
    {1498,1533,1518,1514},
    {1498,1540,1477,1564},
    {1492,1539,1502,1554},
    {1524,1499,1499,1561},
    {1503,1514,1444,1605},
    {1512,1509,1502,1575},
    {1500,1513,1518,1554},
    {1525,1503,1486,1583},
    {1514,1546,1529,1558},
    {1500,1557,1471,1595},
    {1523,1549,1520,1567},
    {1517,1552,1504,1587},
    {1494,1563,1504,1591},
    {1520,1545,1566,1543},
    {1536,1537,1482,1617},
    {1523,1555,1523,1584},
    {1544,1530,1508,1589},
    {1499,1569,1475,1612},
    {1529,1538,1492,1603},
    {1501,1559,1539,1569},
    {1509,1560,1481,1623},
    {1508,1588,1502,1623},
    {1541,1566,1561,1581},
    {1534,1566,1535,1599},
    {1510,1592,1521,1605},
    {1524,1582,1541,1577},
    {1560,1556,1522,1592},
    {1535,1593,1524,1589},
    {1526,1609,1513,1595},
    {1537,1609,1554,1564},
    {1565,1584,1531,1587},
    {1537,1596,1485,1608},
    {1527,1592,1520,1572},
    {1512,1585,1508,1573},
    {1509,1582,1516,1570},
    {1515,1588,1519,1580},
    {1507,1608,1534,1583},
    {1540,1587,1535,1601},
    {1527,1595,1538,1599},
    {1508,1603,1538,1586},
    {1502,1611,1529,1582},
    {1503,1607,1536,1567},
    {1487,1614,1516,1572},
    {1513,1615,1526,1560},
    {1552,1604,1505,1571},
    {1517,1643,1522,1548},
    {1540,1636,1515,1559},
    {1569,1631,1516,1566},
    {1572,1637,1521,1563},
    {1553,1654,1543,1543},
    {1556,1645,1545,1532},
    {1561,1641,1506,1562},
    {1568,1619,1498,1570},
    {1517,1648,1543,1517},
    {1543,1633,1546,1520},
    {1533,1640,1538,1535},
    {1534,1638,1543,1521},
    {1550,1629,1527,1539},
    {1529,1651,1551,1520},
    {1525,1660,1525,1545},
    {1545,1641,1534,1522},
    {1547,1644,1542,1519},
    {1554,1635,1535,1540},
    {1556,1624,1552,1514},
    {1548,1625,1545,1511},
    {1573,1629,1522,1546},
    {1573,1643,1551,1520},
    {1542,1659,1567,1473},
    {1541,1642,1502,1509},
    {1565,1627,1502,1512},
    {1551,1641,1540,1474},
    {1544,1655,1541,1476},
    {1550,1649,1517,1497},
    {1566,1633,1517,1496},
    {1547,1649,1540,1475},
    {1567,1639,1523,1509},
    {1537,1668,1572,1466},
    {1568,1655,1573,1483},
    {1574,1645,1574,1481},
    {1560,1656,1547,1524},
    {1539,1667,1579,1504},
    {1572,1651,1613,1493},
    {1565,1661,1595,1513},
    {1562,1643,1576,1513},
    {1550,1629,1593,1471},
    {1573,1601,1585,1482},
    {1519,1655,1567,1513},
    {1539,1636,1561,1508},
    {1559,1618,1569,1508},
    {1564,1614,1576,1527},
    {1570,1603,1584,1534},
    {1563,1596,1609,1527},
    {1530,1615,1625,1539},
    {1515,1624,1630,1563},
    {1533,1599,1645,1546},
    {1524,1607,1628,1573},
    {1483,1632,1659,1529},
    {1517,1590,1653,1517},
    {1521,1597,1641,1551},
    {1562,1564,1651,1560},
    {1514,1595,1668,1545},
    {1468,1588,1681,1587},
    {1490,1536,1681,1616},
    {1502,1530,1707,1594},
    {1504,1555,1692,1589},
    {1525,1554,1672,1593},
    {1543,1546,1661,1578},
    {1511,1577,1659,1553},
    {1529,1572,1634,1584},
    {1545,1569,1644,1575},
    {1510,1600,1653,1569},
    {1517,1580,1651,1537},
    {1509,1588,1637,1547},
    {1521,1555,1625,1546},
    {1523,1557,1646,1524},
    {1418,1642,1651,1527},
    {1485,1586,1632,1558},
    {1527,1549,1631,1567},
    {1489,1604,1660,1565},
    {1463,1639,1685,1554},
    {1510,1601,1673,1574},
    {1538,1573,1670,1580},
    {1487,1606,1696,1541},
    {1468,1611,1676,1537},
    {1538,1541,1627,1579},
    {1508,1574,1624,1570},
    {1439,1655,1651,1528},
    {1542,1582,1646,1524},
    {1498,1617,1665,1506},
    {1490,1622,1657,1534},
    {1525,1578,1637,1564},
    {1511,1594,1670,1543},
    {1475,1632,1682,1550},
    {1504,1594,1672,1561},
    {1516,1588,1674,1568},
    {1542,1558,1649,1591},
    {1518,1586,1662,1578},
    {1506,1595,1674,1552},
    {1493,1594,1657,1540},
    {1495,1577,1630,1540},
    {1513,1567,1638,1525},
    {1513,1556,1620,1544},
    {1453,1596,1648,1506},
    {1517,1539,1599,1550},
    {1539,1548,1614,1543},
    {1476,1626,1631,1545},
    {1532,1589,1662,1530},
    {1508,1634,1694,1527},
    {1511,1616,1663,1559},
    {1525,1588,1664,1554},
    {1535,1568,1659,1562},
    {1503,1634,1687,1553},
    {1538,1596,1680,1555},
    {1558,1602,1692,1574},
    {1554,1601,1688,1598},
    {1485,1642,1703,1595},
    {1515,1627,1713,1606},
    {1536,1617,1687,1623},
    {1553,1597,1701,1601},
    {1543,1614,1704,1598},
    {1569,1603,1686,1606},
    {1571,1628,1690,1607},
    {1586,1622,1697,1616},
    {1581,1596,1677,1626},
    {1562,1595,1685,1611},
    {1549,1598,1700,1607},
    {1518,1625,1729,1588},
    {1557,1613,1718,1634},
    {1551,1640,1740,1621},
    {1540,1671,1735,1641},
    {1548,1644,1731,1608},
    {1519,1642,1714,1597},
    {1497,1638,1708,1584},
    {1518,1609,1688,1582},
    {1520,1616,1669,1592},
    {1525,1576,1646,1582},
    {1524,1605,1667,1580},
    {1546,1613,1675,1597},
    {1550,1615,1701,1578},
    {1551,1593,1679,1591},
    {1521,1597,1689,1578},
    {1522,1560,1667,1586},
    {1496,1557,1683,1529},
    {1492,1564,1656,1515},
    {1533,1594,1644,1556},
    {1538,1619,1656,1559},
    {1556,1612,1671,1560},
    {1534,1613,1682,1563},
    {1473,1643,1713,1533},
    {1495,1601,1700,1549},
    {1522,1591,1718,1542},
    {1482,1614,1669,1590},
    {1502,1621,1704,1575},
    {1530,1622,1700,1605},
    {1557,1599,1684,1621},
    {1576,1623,1703,1633},
    {1542,1665,1739,1614},
    {1574,1639,1728,1651},
    {1573,1635,1749,1646},
    {1537,1635,1741,1640},
    {1570,1587,1729,1643},
    {1527,1634,1715,1639},
    {1540,1638,1702,1641},
    {1573,1614,1702,1616},
    {1563,1660,1708,1625},
    {1549,1674,1720,1604},
    {1575,1662,1717,1636},
    {1601,1675,1746,1647},
    {1624,1655,1737,1668},
    {1568,1671,1735,1646},
    {1599,1641,1705,1671},
    {1590,1647,1740,1623},
    {1575,1661,1727,1638},
    {1597,1651,1733,1638},
    {1559,1667,1726,1628},
    {1565,1646,1711,1623},
    {1548,1646,1716,1592},
    {1541,1634,1701,1593},
    {1506,1621,1685,1570},
    {1530,1564,1671,1586},
    {1484,1620,1707,1572},
    {1538,1591,1686,1601},
    {1529,1616,1704,1594},
    {1561,1599,1680,1628},
    {1566,1607,1708,1610},
    {1590,1591,1702,1623},
    {1583,1594,1710,1622},
    {1518,1639,1715,1620},
    {1525,1616,1697,1632},
    {1542,1605,1706,1612},
    {1579,1610,1677,1652},
    {1553,1624,1708,1601},
    {1543,1623,1694,1612},
    {1517,1624,1686,1614},
    {1531,1603,1710,1591},
    {1511,1626,1706,1599},
    {1583,1600,1722,1599},
    {1554,1635,1697,1633},
    {1545,1629,1699,1630},
    {1577,1608,1690,1648},
    {1575,1620,1745,1587},
    {1547,1643,1720,1634},
    {1572,1626,1710,1660},
    {1549,1611,1729,1585},
    {1466,1520,1625,1551},
    {1396,1504,1566,1467},
    {1406,1499,1520,1429},
    {1421,1497,1510,1404},
    {1450,1503,1523,1425},
    {1409,1555,1574,1403},
    {1505,1472,1525,1461},
    {1491,1527,1557,1436},
    {1441,1564,1560,1416},
    {1425,1574,1567,1386},
    {1434,1548,1513,1414},
    {1484,1523,1507,1412},
    {1399,1579,1510,1385},
    {1414,1552,1520,1375},
    {1429,1510,1493,1394},
    {1428,1518,1554,1334},
    {1403,1525,1432,1475},
    {1397,1538,1548,1337},
    {1445,1508,1535,1354},
    {1401,1549,1540,1332},
    {1386,1576,1518,1379},
    {1428,1533,1507,1396},
    {1398,1541,1548,1335},
    {1407,1530,1507,1399},
    {1445,1506,1521,1396},
    {1447,1502,1531,1366},
    {1418,1518,1520,1368},
    {1378,1559,1472,1445},
    {1447,1525,1493,1448},
    {1455,1532,1593,1329},
    {1382,1541,1523,1406},
    {1363,1559,1493,1452},
    {1393,1533,1576,1363},
    {1434,1468,1501,1424},
    {1439,1498,1497,1443},
    {1401,1529,1516,1406},
    {1463,1491,1505,1444},
    {1441,1491,1529,1421},
    {1411,1532,1567,1398},
    {1456,1515,1525,1475},
    {1468,1501,1515,1459},
    {1407,1563,1565,1360},
    {1392,1564,1516,1374},
    {1460,1521,1492,1439},
    {1432,1530,1559,1372},
    {1430,1507,1539,1380},
    {1432,1472,1526,1400},
    {1364,1514,1541,1393},
    {1417,1479,1510,1440},
    {1421,1489,1562,1380},
    {1392,1531,1549,1419},
    {1461,1508,1536,1446},
    {1460,1520,1561,1415},
    {1464,1506,1548,1417},
    {1411,1555,1528,1446},
    {1450,1552,1555,1435},
    {1468,1525,1535,1438},
    {1461,1543,1563,1400},
    {1448,1538,1543,1430},
    {1461,1531,1569,1411},
    {1455,1518,1564,1409},
    {1454,1536,1567,1427},
    {1458,1540,1581,1413},
    {1443,1535,1551,1419},
    {1481,1510,1518,1462},
    {1462,1547,1577,1405},
    {1452,1549,1583,1386},
    {1484,1506,1527,1448},
    {1438,1548,1553,1420},
    {1491,1518,1546,1460},
    {1475,1526,1554,1462},
    {1448,1521,1554,1434},
    {1479,1507,1556,1432},
    {1480,1523,1554,1440},
    {1450,1565,1567,1439},
    {1477,1561,1574,1430},
    {1529,1532,1554,1479},
    {1526,1557,1576,1493},
    {1487,1587,1627,1451},
    {1521,1541,1586,1523},
    {1497,1560,1611,1523},
    {1515,1526,1615,1515},
    {1481,1543,1621,1498},
    {1471,1552,1612,1499},
    {1481,1541,1607,1476},
    {1515,1502,1553,1513},
    {1493,1553,1586,1475},
    {1496,1549,1576,1479},
    {1465,1556,1585,1449},
    {1503,1518,1565,1472},
    {1484,1545,1554,1481},
    {1451,1573,1560,1459},
    {1487,1538,1571,1431},
    {1484,1533,1550,1456},
    {1512,1518,1535,1496},
    {1484,1534,1578,1468},
    {1507,1501,1567,1488},
    {1482,1536,1591,1474},
    {1510,1520,1574,1495},
    {1541,1508,1559,1515},
    {1499,1565,1596,1480},
    {1519,1524,1560,1515},
    {1505,1517,1588,1493},
    {1492,1524,1582,1503},
    {1497,1520,1583,1489},
    {1519,1509,1544,1519},
    {1501,1534,1584,1461},
    {1487,1568,1589,1465},
    {1515,1542,1567,1500},
    {1528,1520,1565,1502},
    {1499,1541,1587,1471},
    {1500,1544,1581,1484},
    {1527,1524,1588,1484},
    {1520,1532,1574,1496},
    {1511,1543,1569,1504},
    {1517,1533,1572,1500},
    {1521,1525,1571,1489},
    {1519,1533,1566,1491},
    {1529,1533,1565,1491},
    {1532,1530,1562,1494},
    {1536,1530,1567,1493},
    {1552,1519,1565,1507},
    {1534,1548,1575,1506},
    {1546,1532,1551,1527},
    {1557,1528,1542,1533},
    {1559,1538,1551,1526},
    {1540,1563,1579,1496},
    {1554,1542,1556,1527},
    {1555,1562,1554,1516},
    {1593,1564,1513,1501},
    {1599,1587,1522,1475},
    {1596,1558,1517,1500},
    {1589,1530,1534,1525},
    {1546,1542,1554,1532},
    {1547,1514,1526,1578},
    {1550,1500,1551,1566},
    {1525,1514,1537,1573},
    {1558,1477,1515,1575},
    {1532,1520,1547,1538},
    {1549,1495,1532,1535},
    {1548,1505,1528,1538},
    {1539,1514,1542,1518},
    {1555,1498,1517,1538},
    {1551,1510,1503,1547},
    {1562,1513,1505,1548},
    {1566,1532,1509,1556},
    {1562,1543,1544,1524},
    {1579,1535,1508,1566},
    {1572,1536,1497,1574},
    {1571,1548,1500,1590},
    {1558,1540,1500,1572},
};

// arm_motors_check - checks for pilot input to arm or disarm the copter
// called at 10hz
void Copter::arm_motors_check()
{
    static int16_t arming_counter;

    // ensure throttle is down
    if (channel_throttle->get_control_in() > 0) {
        arming_counter = 0;
        return;
    }

    int16_t tmp = channel_yaw->get_control_in();

    // full right
    if (tmp > 4000) {

        // increase the arming counter to a maximum of 1 beyond the auto trim counter
        if( arming_counter <= AUTO_TRIM_DELAY ) {
            arming_counter++;
        }

        // arm the motors and configure for flight
        if (arming_counter == ARM_DELAY && !motors->armed()) {
            // reset arming counter if arming fail
            if (!init_arm_motors(false)) {
                arming_counter = 0;
            }
        }

        // arm the motors and configure for flight
        if (arming_counter == AUTO_TRIM_DELAY && motors->armed() && control_mode == STABILIZE) {
            auto_trim_counter = 250;
            // ensure auto-disarm doesn't trigger immediately
            auto_disarm_begin = millis();
        }

    // full left
    }else if (tmp < -4000) {
        if (!mode_has_manual_throttle(control_mode) && !ap.land_complete) {
            arming_counter = 0;
            return;
        }

        // increase the counter to a maximum of 1 beyond the disarm delay
        if( arming_counter <= DISARM_DELAY ) {
            arming_counter++;
        }

        // disarm the motors
        if (arming_counter == DISARM_DELAY && motors->armed()) {
            init_disarm_motors();
        }

    // Yaw is centered so reset arming counter
    }else{
        arming_counter = 0;
    }
}

// auto_disarm_check - disarms the copter if it has been sitting on the ground in manual mode with throttle low for at least 15 seconds
void Copter::auto_disarm_check()
{
    uint32_t tnow_ms = millis();
    uint32_t disarm_delay_ms = 1000*constrain_int16(g.disarm_delay, 0, 127);

    // exit immediately if we are already disarmed, or if auto
    // disarming is disabled
    if (!motors->armed() || disarm_delay_ms == 0 || control_mode == THROW) {
        auto_disarm_begin = tnow_ms;
        return;
    }

#if FRAME_CONFIG == HELI_FRAME
    // if the rotor is still spinning, don't initiate auto disarm
    if (motors->rotor_speed_above_critical()) {
        auto_disarm_begin = tnow_ms;
        return;
    }
#endif

    // always allow auto disarm if using interlock switch or motors are Emergency Stopped
    if ((ap.using_interlock && !motors->get_interlock()) || ap.motor_emergency_stop) {
#if FRAME_CONFIG != HELI_FRAME
        // use a shorter delay if using throttle interlock switch or Emergency Stop, because it is less
        // obvious the copter is armed as the motors will not be spinning
        disarm_delay_ms /= 2;
#endif
    } else {
        bool sprung_throttle_stick = (g.throttle_behavior & THR_BEHAVE_FEEDBACK_FROM_MID_STICK) != 0;
        bool thr_low;
        if (mode_has_manual_throttle(control_mode) || !sprung_throttle_stick) {
            thr_low = ap.throttle_zero;
        } else {
            float deadband_top = channel_throttle->get_control_mid() + g.throttle_deadzone;
            thr_low = channel_throttle->get_control_in() <= deadband_top;
        }

        if (!thr_low || !ap.land_complete) {
            // reset timer
            auto_disarm_begin = tnow_ms;
        }
    }

    // disarm once timer expires
    if ((tnow_ms-auto_disarm_begin) >= disarm_delay_ms) {
        init_disarm_motors();
        auto_disarm_begin = tnow_ms;
    }
}

// init_arm_motors - performs arming process including initialisation of barometer and gyros
//  returns false if arming failed because of pre-arm checks, arming checks or a gyro calibration failure
bool Copter::init_arm_motors(bool arming_from_gcs)
{
    static bool in_arm_motors = false;

    // exit immediately if already in this function
    if (in_arm_motors) {
        return false;
    }
    in_arm_motors = true;

    // return true if already armed
    if (motors->armed()) {
        in_arm_motors = false;
        return true;
    }

    // run pre-arm-checks and display failures
    if (!arming.all_checks_passing(arming_from_gcs)) {
        AP_Notify::events.arming_failed = true;
        in_arm_motors = false;
        return false;
    }

    // let dataflash know that we're armed (it may open logs e.g.)
    DataFlash_Class::instance()->set_vehicle_armed(true);

    // disable cpu failsafe because initialising everything takes a while
    failsafe_disable();

    // reset battery failsafe
    set_failsafe_battery(false);

    // notify that arming will occur (we do this early to give plenty of warning)
    AP_Notify::flags.armed = true;
    // call update_notify a few times to ensure the message gets out
    for (uint8_t i=0; i<=10; i++) {
        update_notify();
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Arming motors");
#endif

    // Remember Orientation
    // --------------------
    init_simple_bearing();

    initial_armed_bearing = ahrs.yaw_sensor;

    if (ap.home_state == HOME_UNSET) {
        // Reset EKF altitude if home hasn't been set yet (we use EKF altitude as substitute for alt above home)
        ahrs.resetHeightDatum();
        Log_Write_Event(DATA_EKF_ALT_RESET);
    } else if (ap.home_state == HOME_SET_NOT_LOCKED) {
        // Reset home position if it has already been set before (but not locked)
        set_home_to_current_location(false);
    }
    calc_distance_and_bearing();

    // enable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(true);
    hal.util->set_soft_armed(true);

#if SPRAYER == ENABLED
    // turn off sprayer's test if on
    sprayer.test_pump(false);
#endif

    // enable output to motors
    enable_motor_output();

    // finally actually arm the motors
    motors->armed(true);

    // log arming to dataflash
    Log_Write_Event(DATA_ARMED);

    // log flight mode in case it was changed while vehicle was disarmed
    DataFlash.Log_Write_Mode(control_mode, control_mode_reason);

    // reenable failsafe
    failsafe_enable();

    // perf monitor ignores delay due to arming
    perf_ignore_this_loop();

    // flag exiting this function
    in_arm_motors = false;

    // Log time stamp of arming event
    arm_time_ms = millis();

    // Start the arming delay
    ap.in_arming_delay = true;

    // return success
    return true;
}

// init_disarm_motors - disarm motors
void Copter::init_disarm_motors()
{
    // return immediately if we are already disarmed
    if (!motors->armed()) {
        return;
    }

#if HIL_MODE != HIL_MODE_DISABLED || CONFIG_HAL_BOARD == HAL_BOARD_SITL
    gcs().send_text(MAV_SEVERITY_INFO, "Disarming motors");
#endif

    // save compass offsets learned by the EKF if enabled
    if (ahrs.use_compass() && compass.get_learn_type() == Compass::LEARN_EKF) {
        for(uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
            Vector3f magOffsets;
            if (ahrs.getMagOffsets(i, magOffsets)) {
                compass.set_and_save_offsets(i, magOffsets);
            }
        }
    }

#if AUTOTUNE_ENABLED == ENABLED
    // save auto tuned parameters
    autotune_save_tuning_gains();
#endif

    // we are not in the air
    set_land_complete(true);
    set_land_complete_maybe(true);

    // log disarm to the dataflash
    Log_Write_Event(DATA_DISARMED);

    // send disarm command to motors
    motors->armed(false);

    // reset the mission
    mission.reset();

    DataFlash_Class::instance()->set_vehicle_armed(false);

    // disable gps velocity based centrefugal force compensation
    ahrs.set_correct_centrifugal(false);
    hal.util->set_soft_armed(false);

    ap.in_arming_delay = false;
}

// motors_output - send output to motors library which will adjust and send to ESCs and servos
void Copter::motors_output()
{
#if ADVANCED_FAILSAFE == ENABLED
    // this is to allow the failsafe module to deliberately crash
    // the vehicle. Only used in extreme circumstances to meet the
    // OBC rules
    if (g2.afs.should_crash_vehicle()) {
        g2.afs.terminate_vehicle();
        return;
    }
#endif

    // Update arming delay state
    if (ap.in_arming_delay && (!motors->armed() || millis()-arm_time_ms > ARMING_DELAY_SEC*1.0e3f || control_mode == THROW)) {
        ap.in_arming_delay = false;
    }

    // output any servo channels
    SRV_Channels::calc_pwm();

    // cork now, so that all channel outputs happen at once
    hal.rcout->cork();

    // update output on any aux channels, for manual passthru
    SRV_Channels::output_ch_all();

    // check if we are performing the motor test
    if (ap.motor_test) {
        motor_test_output();
    } else {
        bool interlock = motors->armed() && !ap.in_arming_delay && (!ap.using_interlock || ap.motor_interlock_switch) && !ap.motor_emergency_stop;
        if (!motors->get_interlock() && interlock) {
            motors->set_interlock(true);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_ENABLED);
        } else if (motors->get_interlock() && !interlock) {
            motors->set_interlock(false);
            Log_Write_Event(DATA_MOTORS_INTERLOCK_DISABLED);
        }

        // send output signals to motors
        motors->output();
    }

    if (playback_enabled && hal.util->get_soft_armed() && control_mode == STABILIZE) {
        uint32_t nrows = ARRAY_SIZE(playback);
        static uint32_t last_playback_ms;
        static uint32_t row;
        uint32_t now = AP_HAL::millis();
        if (now - last_playback_ms >= 100) {
            // data is at 10Hz
            row = (row + 1) % nrows;
            last_playback_ms = now;
        }
        for (uint8_t i=0; i<4; i++) {
            hal.rcout->write(i, playback[row].mot[i]);
        }
    }

    // push all channels
    hal.rcout->push();
}

// check for pilot stick input to trigger lost vehicle alarm
void Copter::lost_vehicle_check()
{
    static uint8_t soundalarm_counter;

    // disable if aux switch is setup to vehicle alarm as the two could interfere
    if (check_if_auxsw_mode_used(AUXSW_LOST_COPTER_SOUND)) {
        return;
    }

    // ensure throttle is down, motors not armed, pitch and roll rc at max. Note: rc1=roll rc2=pitch
    if (ap.throttle_zero && !motors->armed() && (channel_roll->get_control_in() > 4000) && (channel_pitch->get_control_in() > 4000)) {
        if (soundalarm_counter >= LOST_VEHICLE_DELAY) {
            if (AP_Notify::flags.vehicle_lost == false) {
                AP_Notify::flags.vehicle_lost = true;
                gcs().send_text(MAV_SEVERITY_NOTICE,"Locate Copter alarm");
            }
        } else {
            soundalarm_counter++;
        }
    } else {
        soundalarm_counter = 0;
        if (AP_Notify::flags.vehicle_lost == true) {
            AP_Notify::flags.vehicle_lost = false;
        }
    }
}
